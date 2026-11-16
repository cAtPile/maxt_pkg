#!/usr/bin/env python3
"""Real BT + QR nodes against a deterministic ROS flight-interface double.

No MAVROS, PX4, Gazebo or hardware driver is launched. Run ONLY on an isolated
loopback master. This validates orchestration, not flight dynamics or accuracy.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
from urllib.parse import urlparse

import rospy
import yaml
from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped
from mavros_msgs.msg import State, PositionTarget, GlobalPositionTarget
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float64, String
from uav_odin_qr.msg import Detection, DetectionArray, QrTransform, QrTransformArray
from uav_qr_mission.msg import MissionResult
from uav_qr_mission.srv import ReportQr, ReportQrResponse
from uav_qr_mission.geolocation import geodetic_to_ecef, offset_wgs84


class FlightDouble:
    def __init__(self, scenario):
        self.scenario = scenario
        self.lock = threading.RLock()
        self.position = [0.,0.,0.]
        self.command = None
        self.captures = []
        self.capture_positions = []
        self.capture_stable_times = []
        self.last_motion = time.monotonic()
        self.latest = None
        self.active = []
        self.global_targets = []
        self.received_reports = []
        self.display = {}
        self.outside_window_violation = False
        self.failure = None
        self.overshoot_injected = False
        self.takeoff_hold_time = None
        self.first_navigation_time = None
        self.climb_after_hold = False
        self.running = True
        self.origin = (31.2,121.4,100.)
        self.ecef_origin = geodetic_to_ecef(*self.origin)
        self.publishers = {}
        for topic, cls in [('/mavros/state',State),('/mavros/local_position/pose',PoseStamped),
                           ('/mavros/local_position/velocity_local',TwistStamped),
                           ('/mavros/global_position/global',NavSatFix),
                           ('/mavros/global_position/raw/fix',NavSatFix),
                           ('/mavros/global_position/rel_alt',Float64),('/mavros/imu/data',Imu),
                           ('/uav_odin_qr/detections',DetectionArray),
                           ('/uav_odin_qr/tf_targets',QrTransformArray)]:
            self.publishers[topic] = rospy.Publisher(topic,cls,queue_size=10)
        self.subs = [
            rospy.Subscriber('/mavros/setpoint_raw/local',PositionTarget,self.local_command),
            rospy.Subscriber('/mavros/setpoint_position/local',PoseStamped,self.pose_command),
            rospy.Subscriber('/mavros/setpoint_raw/global',GlobalPositionTarget,self.global_command),
            rospy.Subscriber('/uav_qr_mission/result',MissionResult,self.result),
            rospy.Subscriber('/maxt_test_node/mission_status',String,self.status)]
        self.service = rospy.Service('/ugv/qr_display/update',ReportQr,self.report)
        self.thread = threading.Thread(target=self.run,daemon=True)
        self.thread.start()

    def local_command(self,message):
        with self.lock:
            self.command = ('raw',message)
            if self.scenario == 'takeoff_overshoot' and self.overshoot_injected:
                if not message.type_mask & PositionTarget.IGNORE_PZ and abs(message.position.z-5) < 1e-6:
                    if self.takeoff_hold_time is None: self.takeoff_hold_time = time.monotonic()
                if (self.takeoff_hold_time is not None and message.type_mask & PositionTarget.IGNORE_PZ
                        and message.velocity.z > 0):
                    self.climb_after_hold = True

    def pose_command(self,message):
        with self.lock: self.command = ('pose',message)

    def global_command(self,message):
        with self.lock:
            self.command = ('global',message)
            self.global_targets.append(message)

    def result(self,message):
        with self.lock:
            if message.capture_id and message.capture_id not in self.captures:
                self.captures.append(message.capture_id)
                self.capture_positions.append(list(self.position))
                self.capture_stable_times.append(time.monotonic()-self.last_motion)
            if any(message.found) and not self.captures:
                self.outside_window_violation = True
            self.latest = message

    def status(self,message):
        with self.lock:
            self.active = json.loads(message.data).get('active',[])
            if self.first_navigation_time is None and any(n.get('name') == 'ToCaptureStop' for n in self.active):
                self.first_navigation_time = time.monotonic()

    def report(self,request):
        with self.lock:
            # Model the screen's idempotent upsert. First response is lost even
            # though the first record was displayed; retry must not duplicate it.
            self.received_reports.append(request.report)
            first = len(self.received_reports) == 1
            for target in request.report.targets:
                assert target.content and target.position_valid and target.position.header.stamp.to_sec() > 0
                key = (request.report.mission_id,target.content)
                if key not in self.display or target.revision >= self.display[key].revision:
                    self.display[key] = target
        if first: time.sleep(2.5)
        return ReportQrResponse(True,'DISPLAYED')

    def to_local(self,lat,lon):
        xyz = geodetic_to_ecef(lat,lon,self.origin[2])
        dx,dy,dz = [a-b for a,b in zip(xyz,self.ecef_origin)]
        lat,lon = map(math.radians,self.origin[:2])
        east = -math.sin(lon)*dx+math.cos(lon)*dy
        north = -math.sin(lat)*math.cos(lon)*dx-math.sin(lat)*math.sin(lon)*dy+math.cos(lat)*dz
        return east,north

    def run(self):
        previous = time.monotonic()
        try:
            while self.running and not rospy.is_shutdown():
                now = time.monotonic()
                dt = min(.05,now-previous)
                previous = now
                with self.lock:
                    before = list(self.position)
                    if self.command:
                        kind,message = self.command
                        if kind == 'pose':
                            self.position = [message.pose.position.x,message.pose.position.y,message.pose.position.z]
                        elif kind == 'raw':
                            for i,axis in enumerate('xyz'):
                                if message.type_mask & (1<<i):
                                    if not message.type_mask & (1<<(i+3)):
                                        self.position[i] += getattr(message.velocity,axis)*dt
                                else: self.position[i] = getattr(message.position,axis)
                        else:
                            self.position[:2] = self.to_local(message.latitude,message.longitude)
                            self.position[2] = message.altitude
                    if self.scenario == 'navigation_failure' and self.captures:
                        self.position[:2] = [0.,0.]
                    if self.scenario == 'takeoff_timeout':
                        self.position[2] = 0.  # altitude never responds to the climb command
                    if (self.scenario == 'takeoff_overshoot' and not self.overshoot_injected
                            and self.command and self.command[0] == 'raw'
                            and self.command[1].type_mask & PositionTarget.IGNORE_PZ
                            and self.position[2] >= 4.8):
                        # Jump across the entire [4.9, 5.1] m tolerance band.
                        self.position[2] = 5.35
                        self.overshoot_injected = True
                    velocity = [(a-b)/max(dt,1e-6) for a,b in zip(self.position,before)]
                    if (self.scenario == 'takeoff_overshoot' and self.takeoff_hold_time is not None
                            and now-self.takeoff_hold_time < .5):
                        # Height has reached 5 m but the velocity estimator still
                        # reports motion. Takeoff must remain RUNNING until it settles.
                        velocity[2] = 1.
                    if sum(v*v for v in velocity) > .2**2: self.last_motion = now
                    self.publish(velocity)
                time.sleep(.02)
        except Exception as error:
            self.failure = error

    def publish(self,velocity):
        stamp = rospy.Time.now()
        state = State(connected=True,armed=True,mode='OFFBOARD')
        self.publishers['/mavros/state'].publish(state)
        pose,twist = PoseStamped(),TwistStamped()
        pose.header.stamp = twist.header.stamp = stamp
        pose.pose.position.x,pose.pose.position.y,pose.pose.position.z = self.position
        pose.pose.orientation.w = 1
        twist.twist.linear.x,twist.twist.linear.y,twist.twist.linear.z = velocity
        self.publishers['/mavros/local_position/pose'].publish(pose)
        self.publishers['/mavros/local_position/velocity_local'].publish(twist)
        self.publishers['/mavros/global_position/rel_alt'].publish(Float64(self.position[2]))
        for topic,offset in [('/mavros/global_position/global',[0,0,0]),
                             ('/mavros/global_position/raw/fix',[0,.1585,.105])]:
            fix = NavSatFix()
            fix.header.stamp = stamp
            enu = [a+b for a,b in zip(self.position,offset)]
            fix.latitude,fix.longitude,fix.altitude = offset_wgs84(*self.origin,enu)
            self.publishers[topic].publish(fix)
        imu = Imu()
        imu.header.stamp = stamp
        imu.orientation.w = 1
        self.publishers['/mavros/imu/data'].publish(imu)
        detections,targets = DetectionArray(),QrTransformArray()
        detections.header.stamp = targets.header.stamp = stamp
        # The scene depends on physical position, never on an asynchronously
        # received capture ACK (otherwise the first image may show the old scene).
        x,y,z = self.position
        if z < 3: codes = ['QR001','QR002','QR003']  # in-flight frames must be ignored
        elif math.hypot(x,y) < 1: codes = ['QR001']
        elif math.hypot(x,y-10) < 1: codes = ['QR002','QR003']
        else: codes = []
        if self.scenario == 'missing_drop': codes = [c for c in codes if c != 'QR003']
        locations = {'QR001':(0,0), 'QR002':(0,10), 'QR003':(2,10)}
        for i,content in enumerate(codes):
            if self.position[2] <= .2: continue
            detections.detections.append(Detection(content=content,center_u=i,center_v=0,depth_valid=True))
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = 'uav_camera_optical_frame'
            tf.child_frame_id = 'synthetic_'+content
            x,y = locations[content]
            tf.transform.translation.x = self.position[1]-y
            tf.transform.translation.y = self.position[0]-.105-x
            tf.transform.translation.z = self.position[2]-.17
            tf.transform.rotation.w = 1
            targets.targets.append(QrTransform(content=content,center_u=i,center_v=0,transform=tf))
        self.publishers['/uav_odin_qr/tf_targets'].publish(targets)
        self.publishers['/uav_odin_qr/detections'].publish(detections)

    def close(self):
        self.running = False
        self.thread.join(timeout=2)


def main():
    args = argparse.ArgumentParser()
    args.add_argument('--scenario',choices=['happy','navigation_failure','missing_drop',
                                          'takeoff_overshoot','takeoff_timeout'],default='happy')
    args.add_argument('--log-dir',required=True)
    options = args.parse_args(rospy.myargv()[1:])
    uri = urlparse(os.environ.get('ROS_MASTER_URI',''))
    if uri.hostname not in ('127.0.0.1','localhost') or uri.port == 11311:
        raise RuntimeError('Use a dedicated non-default loopback master; no vehicle connection allowed')
    rospy.init_node('qr_stop_survey_contract')
    fixture = FlightDouble(options.scenario)
    log_dir = Path(options.log_dir)
    log_dir.mkdir(parents=True,exist_ok=True)
    # All motion speeds below are TEST-ONLY; the fixture teleports along setpoints.
    ascent_speed = 5. if options.scenario == 'takeoff_overshoot' else 1.
    nav_config = yaml.safe_load((Path(__file__).resolve().parents[1]/'config/nav.yaml').read_text())
    nav_config['takeoff']['ascent_speed'] = ascent_speed
    if options.scenario == 'takeoff_timeout': nav_config['takeoff']['timeout'] = .8
    nav_config['nav']['hold_timeout'] = .7
    test_nav_path = log_dir/'nav-test.yaml'
    test_nav_path.write_text(yaml.safe_dump(nav_config))
    log = (log_dir/'launch.log').open('w')
    child = subprocess.Popen(['roslaunch','maxt_pkg','qr_stop_survey.launch',
                              'start_qr_mission:=true','survey_speed:=50',
                              'survey_acceleration:=100','nav_config:='+str(test_nav_path)],
                             stdout=log,stderr=subprocess.STDOUT,
                             start_new_session=True)
    try:
        # Load overrides atomically through launch, not racing persistent ROS params.
        deadline = time.monotonic()+100
        success_scenario = options.scenario in ('happy','takeoff_overshoot')
        final_name = 'ReadyForDrop' if success_scenario else 'SurveyFailureHold'
        while time.monotonic() < deadline:
            with fixture.lock:
                if fixture.failure: raise fixture.failure
                ready = any(n.get('name') == final_name for n in fixture.active)
                if success_scenario and any(n.get('name') == 'SurveyFailureHold' for n in fixture.active):
                    raise AssertionError('Unexpected failure hold; see launch.log')
            if ready: break
            if child.poll() is not None: raise AssertionError('Mission process exited')
            time.sleep(.05)
        else: raise AssertionError('Mission did not reach '+final_name)
        time.sleep(2)
        with fixture.lock:
            assert not fixture.outside_window_violation, 'In-flight observations entered the registry'
            assert all(t >= .95 for t in fixture.capture_stable_times), fixture.capture_stable_times
            expected = [(0,0),(0,10),(0,20),(10,20),(10,10),(10,0),(20,0),(20,10),(20,20)]
            if options.scenario == 'takeoff_timeout':
                assert not fixture.captures and not fixture.global_targets
                assert fixture.first_navigation_time is None
                kind,last_command = fixture.command
                assert kind == 'raw' and not last_command.type_mask & PositionTarget.IGNORE_PZ
                assert last_command.type_mask & PositionTarget.IGNORE_VZ
                assert abs(last_command.position.z) < .1
            elif options.scenario == 'navigation_failure':
                assert len(fixture.captures) == 1, fixture.capture_positions
                assert not fixture.global_targets, 'Failed waypoint incorrectly reached drop navigation'
            else:
                assert len(fixture.captures) == 9, fixture.capture_positions
                for position,point in zip(fixture.capture_positions,expected):
                    assert math.hypot(position[0]-point[0],position[1]-point[1]) < .15
                assert fixture.latest.survey_finished
                if success_scenario:
                    assert fixture.latest.all_localized
                    assert fixture.global_targets
                    assert abs(fixture.global_targets[-1].altitude-5) < 1e-6
                    assert abs(fixture.position[2]-5) < .1
                    assert math.hypot(fixture.position[0]-2,fixture.position[1]-10) < .15
                    assert len(fixture.display) == 3
                    assert any(len(r.targets) == 1 for r in fixture.received_reports)
                    assert any(len(r.targets) > 1 for r in fixture.received_reports)
                    if options.scenario == 'takeoff_overshoot':
                        assert fixture.overshoot_injected and fixture.takeoff_hold_time is not None
                        assert not fixture.climb_after_hold, 'Positive VZ resumed after height capture'
                        assert fixture.first_navigation_time-fixture.takeoff_hold_time >= .85, \
                            'Takeoff succeeded while moving or before stable dwell'
                        assert all(abs(p[2]-5) < .1 for p in fixture.capture_positions), \
                            'Overshoot was adopted as the survey altitude'
                else:
                    assert not fixture.global_targets, 'Missing drop code must never navigate'
                    assert len(fixture.display) == 2, 'Both ordinary codes must still reach the screen'
            summary = dict(scenario=options.scenario,stops=fixture.capture_positions,
                           stable_before_capture=fixture.capture_stable_times,
                           displayed_codes=sorted(t.content for t in fixture.display.values()),
                           batch_sizes=[len(r.targets) for r in fixture.received_reports],
                           targets=[dict(content=t.content,found=t.found,valid=t.position_valid,status=t.status)
                                    for t in fixture.latest.targets],
                           final_position=fixture.position,active=fixture.active)
            if options.scenario == 'takeoff_overshoot':
                summary['takeoff_hold_before_navigation_s'] = fixture.first_navigation_time-fixture.takeoff_hold_time
        (log_dir/'summary.json').write_text(json.dumps(summary,indent=2))
        print('PASS:',options.scenario,json.dumps(summary))
    finally:
        os.killpg(child.pid,signal.SIGINT)
        try: child.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(child.pid,signal.SIGKILL)
            child.wait()
        fixture.close()
        log.close()
        rospy.signal_shutdown('Contract complete')


if __name__ == '__main__':
    main()
