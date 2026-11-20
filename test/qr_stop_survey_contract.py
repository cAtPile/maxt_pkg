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
import struct
import subprocess
import threading
import time
from urllib.parse import urlparse

import rospy
import yaml
from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped
from mavros_msgs.msg import State, PositionTarget, GlobalPositionTarget, Mavlink, TimesyncStatus
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float64, String
from uav_odin_qr.msg import Detection, DetectionArray, QrTransform, QrTransformArray
from uav_qr_mission.msg import MissionResult
from uav_qr_mission.srv import ReportQr, ReportQrResponse
from uav_qr_mission.geolocation import geodetic_to_ecef, offset_wgs84


class FlightDouble:
    def __init__(self, scenario):
        self.scenario = scenario
        roles_path = Path(__file__).resolve().parents[1]/'config/qr_stop_survey.yaml'
        roles = yaml.safe_load(roles_path.read_text())['qr_survey']
        self.ordinary_ids = roles['report_id']
        self.drop_id = roles['target_id']
        self.field_profile = scenario == 'field_5m'
        self.survey_height = 3. if self.field_profile else 5.
        self.drop_location = (1.,1.) if self.field_profile else (2.,10.)
        self.lock = threading.RLock()
        self.signed_heading = scenario == 'signed_heading'
        self.position = [12.,-7.,0.] if self.signed_heading else [0.,0.,0.]
        self.wait_arm_seen = None
        self.boot_origin_ns = rospy.Time.now().to_nsec()-10000000000
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
                           ('/mavlink/from',Mavlink),('/mavros/timesync_status',TimesyncStatus),
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
            if self.wait_arm_seen is None and any(n.get('name') == 'Takeoff' for n in self.active):
                self.wait_arm_seen = time.monotonic()
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
        armed = not self.signed_heading or (self.wait_arm_seen is not None and time.monotonic()-self.wait_arm_seen>1.2)
        yaw = math.pi/2 if self.signed_heading and self.wait_arm_seen is not None and time.monotonic()-self.wait_arm_seen>.6 else 0.
        # Simulate an attitude change after the first stop: region axes must stay frozen.
        if self.signed_heading and self.captures:
            yaw = -math.pi/3
        state = State(connected=True,armed=armed,mode='OFFBOARD')
        boot_ms = (stamp.to_nsec()-self.boot_origin_ns)//1000000
        payload = struct.pack('<QI',stamp.to_nsec()//1000,boot_ms)
        wire = Mavlink(framing_status=Mavlink.FRAMING_OK,sysid=1,compid=1,msgid=2,len=12,
                       payload64=list(struct.unpack('<QQ',payload.ljust(16,b'\0'))))
        self.publishers['/mavlink/from'].publish(wire)
        self.publishers['/mavros/timesync_status'].publish(TimesyncStatus(estimated_offset_ns=self.boot_origin_ns,round_trip_time_ms=1))
        self.publishers['/mavros/state'].publish(state)
        pose,twist = PoseStamped(),TwistStamped()
        pose.header.stamp = twist.header.stamp = stamp
        pose.pose.position.x,pose.pose.position.y,pose.pose.position.z = self.position
        pose.pose.orientation.z = math.sin(yaw/2)
        pose.pose.orientation.w = math.cos(yaw/2)
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
        imu.orientation.z = math.sin(yaw/2)
        imu.orientation.w = math.cos(yaw/2)
        self.publishers['/mavros/imu/data'].publish(imu)
        detections,targets = DetectionArray(),QrTransformArray()
        detections.header.stamp = targets.header.stamp = stamp
        # The scene depends on physical position, never on an asynchronously
        # received capture ACK (otherwise the first image may show the old scene).
        x,y,z = self.position
        radius = .4 if self.field_profile else 1.
        second_y = 1. if self.field_profile else 10.
        if z < self.survey_height/2: codes = self.ordinary_ids+[self.drop_id]  # in-flight frames must be ignored
        elif math.hypot(x,y) < radius: codes = self.ordinary_ids[:1]
        elif math.hypot(x,y-second_y) < radius: codes = self.ordinary_ids[1:]+[self.drop_id]
        else: codes = []
        if self.signed_heading: codes = []
        if self.scenario == 'missing_drop': codes = [c for c in codes if c != self.drop_id]
        locations = {self.ordinary_ids[0]:(0,0), self.ordinary_ids[1]:(0,second_y),
                     self.drop_id:self.drop_location}
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
                                          'takeoff_overshoot','takeoff_timeout','field_5m','signed_heading'],default='happy')
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
    # Keep the original regression scenarios independent of deployment defaults.
    # field_5m explicitly keeps its historical 5m / 1m / 3m regression profile.
    survey_config = yaml.safe_load((Path(__file__).resolve().parents[1]/'config/qr_stop_survey.yaml').read_text())
    if not fixture.field_profile:
        survey_config['qr_survey'].update(corner_x=20.,corner_y=20.,step=10.,
                                         survey_height=5.,drop_height=5.)
    if fixture.field_profile:
        survey_config['qr_survey'].update(corner_x=5.,corner_y=5.,step=1.,survey_height=3.,drop_height=3.)
    if fixture.signed_heading:
        survey_config['qr_survey'].update(corner_x=5.,corner_y=-5.,step=2.)
    test_survey_path = log_dir/'survey-test.yaml'
    test_survey_path.write_text(yaml.safe_dump(survey_config))
    log = (log_dir/'launch.log').open('w')
    child = subprocess.Popen(['roslaunch','maxt_pkg','qr_stop_survey.launch',
                              'start_qr_mission:=true','start_report_sender:=true','survey_speed:=50',
                              'survey_acceleration:=100','nav_config:='+str(test_nav_path),
                              'survey_config:='+str(test_survey_path)],
                             stdout=log,stderr=subprocess.STDOUT,
                             start_new_session=True)
    try:
        # Load overrides atomically through launch, not racing persistent ROS params.
        deadline = time.monotonic()+(180 if fixture.field_profile else 100)
        success_scenario = options.scenario in ('happy','takeoff_overshoot','field_5m')
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
            if fixture.field_profile:
                expected = [(x,y) for x in range(6)
                            for y in (range(6) if x%2 == 0 else reversed(range(6)))]
            if fixture.signed_heading:
                grid=[(x,y) for i,x in enumerate([0,2,4,5])
                      for y in ([0,-2,-4,-5] if i%2==0 else [-5,-4,-2,0])]
                expected=[(12-y,-7+x) for x,y in grid]
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
                assert len(fixture.captures) == len(expected), fixture.capture_positions
                for position,point in zip(fixture.capture_positions,expected):
                    assert math.hypot(position[0]-point[0],position[1]-point[1]) < .15
                assert fixture.latest.survey_finished
                if success_scenario:
                    assert fixture.latest.position_target_id == fixture.drop_id
                    assert any(t.content == fixture.drop_id and t.is_drop_target and t.position_valid
                               for t in fixture.latest.targets)
                    assert fixture.latest.all_localized
                    assert fixture.global_targets
                    assert abs(fixture.global_targets[-1].altitude-fixture.survey_height) < 1e-6
                    assert abs(fixture.position[2]-fixture.survey_height) < .1
                    assert math.hypot(fixture.position[0]-fixture.drop_location[0],
                                      fixture.position[1]-fixture.drop_location[1]) < .15
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
                    assert not next(t for t in fixture.latest.targets if t.content == fixture.drop_id).position_valid
                    assert len(fixture.display) == (0 if fixture.signed_heading else 2), 'Both ordinary codes must still reach the screen'
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
