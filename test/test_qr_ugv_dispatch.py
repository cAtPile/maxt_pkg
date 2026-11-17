#!/usr/bin/env python3
"""Real QR adapter + C++ client + harmless BT. No flight nodes or hardware."""
import copy
import fcntl
import hashlib
import json
import os
import signal
import subprocess
import tempfile
import threading
import time
import unittest

import rospy
import rostest
from rosgraph_msgs.msg import Log
from std_msgs.msg import String, UInt64, Float64
from slicer_air_ground_msgs.msg import Command
from uav_qr_mission.msg import MissionResult, QrTargetResult


def until(predicate, timeout=4.):
    end = time.monotonic()+timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(.01)
    raise AssertionError('condition timed out')


class DispatchTests(unittest.TestCase):
    def setUp(self):
        self.ns = '/test_dispatch/' + self._testMethodName
        self.lock = threading.RLock()
        self.commands, self.logs, self.ticks, self.durations = [], [], [], []
        self.message = None
        self.offset, self.worker_offset = 0., 0.
        self.publish_state = True
        self.bad_state = None
        self.mode = 'normal'
        self.state = dict(protocol_version=1, server_id='fake-ugv-boot', last_sequence=None,
                          task_id=None, phase='IDLE', runtime_phase='IDLE', startup_checked=True,
                          cancellation_pending=False, datum='WGS84', physical_reference='base_link_horizontal',
                          last_command=None, last_task_result=None, reason='')
        self.state_pub = rospy.Publisher(self.ns+'/state', String, queue_size=1, latch=True)
        self.qr_pub = rospy.Publisher(self.ns+'/qr/result', MissionResult, queue_size=1, latch=True)
        self.begin_pub = rospy.Publisher(self.ns+'/probe/begin', String, queue_size=1)
        self.subscribers = [
            rospy.Subscriber(self.ns+'/command', Command, self.receive, queue_size=10),
            rospy.Subscriber('/rosout', Log, self.log, queue_size=100),
            rospy.Subscriber(self.ns+'/probe/bt_ticks', UInt64,
                             lambda m: self.ticks.append((time.monotonic(), m.data)), queue_size=100),
            rospy.Subscriber(self.ns+'/probe/bt_tick_seconds', Float64,
                             lambda m: self.durations.append(m.data), queue_size=100)]
        self.timer = rospy.Timer(rospy.Duration(.025), self.publish)
        self.output = tempfile.TemporaryFile(mode='w+')
        command = ['rosrun', 'maxt_pkg', 'qr_ugv_dispatch_probe', '__name:=probe', '__ns:='+self.ns,
                   '/air_ground/command:='+self.ns+'/command', '/air_ground/state:='+self.ns+'/state',
                   '_qr_survey/mission_namespace:='+self.ns+'/qr',
                   '_ugv_dispatch/state_timeout:=0.25', '_ugv_dispatch/connect_timeout:=0.25',
                   '_ugv_dispatch/receipt_timeout:=0.3', '_ugv_dispatch/state_max_age:=0.2',
                   '_ugv_dispatch/execution_max_age:=0.2']
        self.child = subprocess.Popen(command, stdout=self.output, stderr=subprocess.STDOUT,
                                      start_new_session=True)
        until(lambda: self.qr_pub.get_num_connections() > 0 and self.begin_pub.get_num_connections() > 0)
        until(lambda: len(self.ticks) >= 5)

    def tearDown(self):
        self.timer.shutdown()
        os.killpg(self.child.pid, signal.SIGINT)
        try:
            self.child.wait(timeout=4.)
        except subprocess.TimeoutExpired:
            os.killpg(self.child.pid, signal.SIGKILL)
            self.child.wait(timeout=2.)
        if self._outcome.errors and any(error for _, error in self._outcome.errors):
            self.output.seek(0)
            print(self.output.read())
        self.output.close()
        for subscriber in self.subscribers:
            subscriber.unregister()
        for publisher in (self.state_pub, self.qr_pub, self.begin_pub):
            publisher.unregister()

    def publish(self, _):
        with self.lock:
            now = rospy.Time.now()
            if self.publish_state:
                self.state['updated_stamp'] = now.to_sec()+self.offset
                self.state['execution_updated_stamp'] = now.to_sec()+self.worker_offset
                self.state_pub.publish(String(data=self.bad_state or json.dumps(self.state)))
            if self.message is not None:
                self.message.header.stamp = now
                self.qr_pub.publish(self.message)

    def log(self, message):
        if message.name == self.ns+'/probe':
            self.logs.append(message.msg)

    def has_log(self, text):
        return any(text in message for message in self.logs)

    def receive(self, message):
        with self.lock:
            self.commands.append(message)
            if self.mode == 'drop':
                return
            receipt = {name: getattr(message, name) for name in
                       ('sequence', 'task_id', 'request_id', 'operation')}
            receipt.update(accepted=self.mode != 'reject', reason='fake rejection', duplicate=False)
            self.state.update(last_command=receipt, last_sequence=message.sequence)
            if receipt['accepted']:
                self.state.update(task_id=message.task_id, phase='READY', runtime_phase='READY')

    def start(self, mission='mission', target='QR001'):
        self.begin_pub.publish(String(data=mission+'|'+target))
        time.sleep(.04)

    def qr(self, content='QR001', mission='mission', valid=True, frame='wgs84', stamp=True, revision=1):
        record = QrTargetResult(content=content, is_drop_target=content == 'QR001', found=True,
                                position_valid=valid, revision=revision)
        record.position.header.frame_id = frame
        if stamp:
            record.position.header.stamp = rospy.Time.now()
        record.position.latitude, record.position.longitude = 30.123456789, 120.987654321
        record.position.altitude = 999.  # Must never become a UGV goal height.
        result = MissionResult(mission_id=mission, position_target_id='QR001', targets=[record])
        with self.lock:
            self.message = result

    def sent(self):
        self.start()
        self.qr()
        until(lambda: len(self.commands) == 1)
        return self.commands[0]

    def complete(self, message, valid=True):
        now = rospy.Time.now().to_sec()
        result = dict(target=dict(task_id=message.task_id), move_base=dict(state=3),
                      stop=dict(confirmed=valid, window_start_stamp=now-1, window_end_stamp=now),
                      geographic=dict(confirmed=True, qualified=True, horizontal_residual_m=.1,
                                      tolerance_m=.35, stamp=now-.1,
                                      stopped_window=dict(start_stamp=now-1, end_stamp=now)))
        with self.lock:
            self.state.update(phase='COMPLETED', runtime_phase='COMPLETED', last_task_result=dict(
                task_id=message.task_id, status='COMPLETED', reason='', result=result))

    def assert_bt_alive(self):
        before = len(self.ticks)
        time.sleep(.2)
        self.assertGreater(len(self.ticks)-before, 8)
        self.assertLess(max(self.durations), .05)

    def test_dispatch_logs_execution_completion_and_dedup(self):
        message = self.sent()
        self.assertEqual(message.operation, 'SET_TASK')
        self.assertEqual(message.sequence, 1)
        self.assertEqual(message.server_id, 'fake-ugv-boot')
        self.assertAlmostEqual(message.final_goal.latitude, 30.123456789, places=9)
        self.assertAlmostEqual(message.final_goal.longitude, 120.987654321, places=9)
        self.assertFalse(message.final_goal.has_altitude)
        self.assertEqual(message.final_goal.altitude_ellipsoid_m, 0.)
        self.assertFalse(message.final_goal.has_yaw)
        self.assertEqual(message.via_points, [])
        self.assertEqual(message._connection_header.get('latching'), '0')
        until(lambda: self.has_log('已发送【QR001】') and self.has_log('任务已受理'))
        self.assertTrue(self.has_log('30.123456789, 120.987654321'))
        self.assertFalse(self.has_log('开始执行任务'))  # READY is acceptance only.
        with self.lock:
            self.state.update(phase='EXECUTING', runtime_phase='NAVIGATING')
        until(lambda: self.has_log('UGV 开始执行任务'))
        self.complete(message)
        until(lambda: self.has_log('UGV 任务完成！'))
        time.sleep(.2)
        self.assertEqual(len(self.commands), 1)
        self.assert_bt_alive()

    def test_unarmed_adapter_ignores_old_mission_and_ordinary_codes(self):
        self.qr(mission='old-mission')
        time.sleep(.15)
        self.assertEqual(self.commands, [])
        self.start()
        for content, mission in [('QR001', 'old-mission'), ('QR002', 'mission'), ('QR003', 'mission')]:
            self.qr(content=content, mission=mission)
            time.sleep(.1)
            self.assertEqual(self.commands, [])
        self.qr()
        until(lambda: len(self.commands) == 1)

    def test_invalid_ground_points_are_not_sent(self):
        self.start()
        for kwargs in ({'valid': False}, {'frame': 'map'}, {'stamp': False}, {'revision': 0}):
            self.qr(**kwargs)
            time.sleep(.1)
            self.assertEqual(self.commands, [])
        self.qr()
        with self.lock:
            self.message.targets[0].position.latitude = float('nan')
        time.sleep(.12)
        self.assertEqual(self.commands, [])
        self.qr()
        until(lambda: len(self.commands) == 1)

    def test_no_vehicle_state_does_not_block_bt_or_publish(self):
        with self.lock:
            self.publish_state = False
        time.sleep(.25)
        self.start()
        self.qr()
        until(lambda: self.has_log('未发送【QR001】'))
        self.assertEqual(self.commands, [])
        self.assert_bt_alive()

    def test_stale_worker_does_not_publish(self):
        with self.lock:
            self.worker_offset = -10.
        self.start()
        self.qr()
        until(lambda: self.has_log('未发送【QR001】'))
        self.assertEqual(self.commands, [])
        self.assert_bt_alive()

    def test_future_state_is_refused(self):
        with self.lock:
            self.offset = 10.
        self.start()
        self.qr()
        until(lambda: self.has_log('未发送【QR001】'))
        self.assertEqual(self.commands, [])

    def test_duplicate_json_keys_are_refused(self):
        with self.lock:
            value = dict(self.state, updated_stamp=rospy.Time.now().to_sec(),
                         execution_updated_stamp=rospy.Time.now().to_sec())
            self.bad_state = json.dumps(value)[:-1]+', "protocol_version": 1}'
        time.sleep(.06)
        self.start()
        self.qr()
        until(lambda: self.has_log('未发送【QR001】'))
        self.assertEqual(self.commands, [])

    def test_malformed_completion_keeps_callback_and_bt_alive(self):
        message = self.sent()
        with self.lock:
            self.state['last_task_result'] = dict(task_id=message.task_id, status='COMPLETED',
                                                   result=dict(target='invalid object'))
        until(lambda: self.has_log('无效反馈/本地异常'))
        self.assertFalse(self.has_log('UGV 任务完成！'))
        self.assert_bt_alive()

    def test_receipt_identity_mismatch_is_not_accepted(self):
        self.mode = 'drop'
        message = self.sent()
        with self.lock:
            self.state['last_command'] = dict(request_id=message.request_id, task_id=message.task_id,
                                              operation='SET_TASK', sequence=message.sequence+1,
                                              accepted=True)
        until(lambda: self.has_log('请求结果未知'))
        self.assertFalse(self.has_log('任务已受理'))
        self.assertEqual(len(self.commands), 1)

    def test_other_task_result_cannot_complete_this_task(self):
        message = self.sent()
        with self.lock:
            self.complete(message)
            self.state['last_task_result']['task_id'] = 'another-task'
        time.sleep(.15)
        self.assertFalse(self.has_log('UGV 任务完成！'))
        self.complete(message)
        until(lambda: self.has_log('UGV 任务完成！'))

    def test_no_command_subscriber_does_not_publish(self):
        self.subscribers[0].unregister()
        time.sleep(.1)
        self.start()
        self.qr()
        until(lambda: self.has_log('未发送【QR001】'))
        self.assertEqual(self.commands, [])
        self.assert_bt_alive()

    def test_receipt_timeout_is_unknown_without_retry(self):
        self.mode = 'drop'
        self.sent()
        until(lambda: self.has_log('请求结果未知'))
        time.sleep(.2)
        self.assertEqual(len(self.commands), 1)
        self.assertFalse(self.has_log('任务完成！'))
        self.assert_bt_alive()

    def test_explicit_rejection_does_not_claim_received(self):
        self.mode = 'reject'
        self.sent()
        until(lambda: self.has_log('UGV 拒绝任务'))
        self.assertFalse(self.has_log('任务已受理'))
        self.assertFalse(self.has_log('任务完成！'))

    def test_blocked_task_does_not_claim_execution_or_release(self):
        self.sent()
        until(lambda: self.has_log('任务已受理'))
        with self.lock:
            self.state.update(phase='BLOCKED', reason='external hold')
        until(lambda: self.has_log('external hold'))
        self.assertFalse(self.has_log('开始执行任务'))
        self.assertEqual([message.operation for message in self.commands], ['SET_TASK'])

    def test_incomplete_arrival_does_not_claim_completed(self):
        message = self.sent()
        self.complete(message, valid=False)
        until(lambda: self.has_log('到达证据不完整'))
        self.assertFalse(self.has_log('UGV 任务完成！'))

    def test_receiver_restart_does_not_resend(self):
        self.sent()
        with self.lock:
            self.state.update(server_id='new-boot', task_id=None, last_sequence=None, last_command=None)
        until(lambda: self.has_log('车端已重启'))
        time.sleep(.15)
        self.assertEqual(len(self.commands), 1)
        self.assert_bt_alive()

    def test_new_mission_has_new_identity_and_sequence(self):
        first = self.sent()
        self.complete(first)
        until(lambda: self.has_log('UGV 任务完成！'))
        self.start('new-mission')
        self.qr(mission='new-mission')
        until(lambda: len(self.commands) == 2)
        second = self.commands[1]
        self.assertEqual(second.sequence, 2)
        self.assertNotEqual(second.request_id, first.request_id)
        self.assertNotEqual(second.task_id, first.task_id)

    def test_python_and_cpp_share_local_sender_lock(self):
        identity = os.environ.get('ROS_MASTER_URI', '')+'\n'+self.ns+'/command'
        path = '/tmp/ag_client-'+str(os.getuid())+'-'+hashlib.sha256(identity.encode()).hexdigest()+'.lock'
        fd = os.open(path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.start()
            self.qr()
            until(lambda: self.has_log('有其他命令出口'))
            self.assertEqual(self.commands, [])
        finally:
            os.close(fd)


if __name__ == '__main__':
    rospy.init_node('qr_ugv_dispatch_test')
    rostest.rosrun('maxt_pkg', 'qr_ugv_dispatch', DispatchTests)
