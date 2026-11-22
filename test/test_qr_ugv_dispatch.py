#!/usr/bin/env python3
"""Exercise the maxt QR adapter through the real V2 gateway with a fake UGV."""
import os
from io import BytesIO
from pathlib import Path
import signal
import subprocess
import time
import unittest

import cv2
import numpy as np
import rospy
import rostest
import yaml
from sensor_msgs.msg import CompressedImage
from slicer_air_ground_msgs.msg import V2Command, V2State
from std_msgs.msg import String, UInt64
from uav_qr_mission.msg import MissionResult, QrTargetResult
from ag_uav_client.v2 import COMMAND, INTENT, STATE, UAV_STATE, validate
from ag_uav_client.v2_ros import command_from_ros


def until(check, timeout=5.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if check():
            return
        time.sleep(.01)
    raise AssertionError('condition timed out')


class DispatchTests(unittest.TestCase):
    def setUp(self):
        self.ns = '/test_dispatch/' + self._testMethodName
        config = Path(__file__).resolve().parents[1] / 'config/qr_stop_survey.yaml'
        roles = yaml.safe_load(config.read_text())['qr_survey']
        self.target = roles['target_id']
        self.report_a, self.report_b = roles['report_id']
        rospy.set_param(self.ns + '/probe/qr_survey', dict(roles, mission_namespace=self.ns + '/qr'))
        self.commands, self.ticks, self.children = [], [], []
        self.latest = None
        self.state_offset = 0.
        self.state_reason = None
        self.outputs = [
            rospy.Publisher(self.ns + '/state', V2State, queue_size=1, latch=True),
            rospy.Publisher(self.ns + '/qr/result', MissionResult, queue_size=1),
            rospy.Publisher(self.ns + '/probe/begin', String, queue_size=1),
        ]
        self.state_pub, self.qr_pub, self.begin_pub = self.outputs
        self.inputs = [
            rospy.Subscriber(self.ns + '/command', V2Command, self.commands.append, queue_size=16),
            rospy.Subscriber(self.ns + '/probe/bt_ticks', UInt64, self.ticks.append, queue_size=100),
        ]
        self.addCleanup(self.cleanup)
        self.timer = rospy.Timer(rospy.Duration(.025), self.publish)
        self.children.append(subprocess.Popen(
            ['rosrun', 'maxt_pkg', 'qr_ugv_dispatch_probe', '__name:=probe', '__ns:=' + self.ns,
             '_ugv_dispatch/state_max_age:=0.4',
             INTENT + ':=' + self.ns + '/intent', STATE + ':=' + self.ns + '/state'],
            start_new_session=True))
        until(lambda: self.qr_pub.get_num_connections() and self.begin_pub.get_num_connections() and
              self.state_pub.get_num_connections() and len(self.ticks) >= 5)

    def start_gateway(self):
        gateway = subprocess.Popen(
            ['rosrun', 'ag_uav_client', 'ag_uav_gateway.py', '__ns:=' + self.ns,
             INTENT + ':=' + self.ns + '/intent', COMMAND + ':=' + self.ns + '/command',
             UAV_STATE + ':=' + self.ns + '/uav_state'], start_new_session=True)
        self.children.append(gateway)
        until(self.inputs[0].get_num_connections)

    def cleanup(self):
        self.timer.shutdown()
        for child in reversed(self.children):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
                try:
                    child.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait(timeout=2.)
        for handle in self.inputs + self.outputs:
            handle.unregister()

    def publish(self, _):
        now = rospy.Time.now()
        if self.state_offset is not None:
            self.state_pub.publish(V2State(version=2, vehicle_id='ugv',
                                           updated_at=now.to_sec() + self.state_offset,
                                           reason=self.state_reason or ''))
        result = self.latest
        if result is not None:
            result.header.stamp = now
            self.qr_pub.publish(result)

    def begin(self, mission='survey'):
        self.begin_pub.publish(String(data=mission))
        time.sleep(.06)

    def record(self, qr_id, valid=True, frame='wgs84', drop=None):
        result = QrTargetResult(content=qr_id, is_drop_target=(qr_id == self.target if drop is None else drop),
                                found=True, position_valid=valid, revision=1)
        result.position.header.frame_id = frame
        result.position.header.stamp = rospy.Time.now()
        result.position.latitude = 31.123456789
        result.position.longitude = 118.987654321
        result.position.altitude = 82.5
        return result

    def result(self, *records, mission='survey', images=()):
        self.latest = MissionResult(mission_id=mission, position_target_id=self.target,
                                    targets=list(records), target_images=list(images))

    def received(self):
        return [validate(command_from_ros(message)) for message in self.commands]

    def test_roles_measurements_and_dedup(self):
        self.start_gateway()
        self.begin()
        first = self.record(self.report_b)
        drop = self.record(self.target)
        images = []
        for record in (first, drop):
            ok, jpeg = cv2.imencode('.jpg', np.full((32, 32), 124, dtype=np.uint8))
            self.assertTrue(ok)
            image = CompressedImage(format='jpeg', data=jpeg.tobytes())
            image.header.stamp = record.position.header.stamp
            image.header.frame_id = 'odin_optical_frame'
            images.append(image)
        self.result(first, images=images[:1])
        until(lambda: len(self.commands) == 1)
        self.result(first, drop, self.record(self.report_a), images=images + [CompressedImage()])
        until(lambda: len(self.commands) == 3)
        time.sleep(.15)
        sent = self.received()
        self.assertEqual([command['op'] for command in sent], ['REPORT_QR', 'SET_TARGET', 'REPORT_QR'])
        self.assertEqual([command['task_id'] for command in sent], ['survey'] * 3)
        self.assertEqual([sent[0]['qr']['qr_id'], sent[2]['qr']['qr_id']], [self.report_b, self.report_a])
        self.assertEqual(sent[1]['target'], dict(lat=31.123456789, lon=118.987654321, image_id=self.target))
        self.assertEqual(sent[0]['qr'], dict(qr_id=self.report_b, lat=31.123456789, lon=118.987654321,
                                             measured_at=first.position.header.stamp.to_sec(),
                                             revision=1, altitude_m=82.5, image_id=self.report_b))
        for command, image in zip(sent, images):
            expected, actual = BytesIO(), BytesIO()
            image.serialize(expected)
            command['image'].serialize(actual)
            self.assertEqual(expected.getvalue(), actual.getvalue())
        self.assertTrue(all(message._connection_header.get('latching') == '0' for message in self.commands))

    def test_ignores_unregistered_wrong_role_and_invalid_results(self):
        self.start_gateway()
        self.begin()
        self.result(self.record(self.target), mission='old-survey')
        time.sleep(.15)
        bad_coordinate = self.record(self.target)
        bad_coordinate.position.latitude = float('nan')
        self.result(self.record('UNREGISTERED'), self.record(self.report_a, drop=True),
                    self.record(self.target, drop=False), self.record(self.report_b, valid=False),
                    self.record(self.report_a, frame='map'), bad_coordinate)
        time.sleep(.2)
        self.assertEqual(self.commands, [])
        self.result(self.record(self.target), self.record(self.report_a))
        until(lambda: len(self.commands) == 2)

    def test_waits_for_fresh_state_and_gateway_without_blocking_bt(self):
        self.begin()
        self.result(self.record(self.target))
        time.sleep(.12)
        self.state_offset = -10.
        time.sleep(.06)
        self.start_gateway()
        before = len(self.ticks)
        time.sleep(.25)
        self.assertEqual(self.commands, [])
        self.assertGreater(len(self.ticks) - before, 10)
        self.state_reason = 'MONITOR_ONLY'
        self.state_offset = 0.
        time.sleep(.15)
        self.assertEqual(self.commands, [])
        self.state_reason = None
        until(lambda: len(self.commands) == 1)
        time.sleep(.15)
        self.assertEqual(len(self.commands), 1)

    def test_new_mission_can_submit_same_codes(self):
        self.start_gateway()
        for mission in ('survey', 'next'):
            self.begin(mission)
            self.result(self.record(self.report_a), self.record(self.target), mission=mission)
            until(lambda: len(self.commands) == (2 if mission == 'survey' else 4))
        self.assertEqual([command['task_id'] for command in self.received()], ['survey'] * 2 + ['next'] * 2)


if __name__ == '__main__':
    rospy.init_node('qr_ugv_dispatch_test')
    rostest.rosrun('maxt_pkg', 'qr_ugv_dispatch', DispatchTests)
