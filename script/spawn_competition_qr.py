#!/usr/bin/env python3
import math
import random
import sys

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose


TARGET_IDS = ('QR002', 'QR003', 'QR004', 'QR005', 'QR006')


def random_positions(center_x, center_y, field_size, panel_size):
    # The bounding circle keeps every rotated corner inside the field.
    radius = panel_size / math.sqrt(2.0)
    half_span = field_size / 2.0 - radius - 0.5
    if not math.isfinite(half_span) or half_span <= 0.0 or panel_size <= 0.0:
        raise ValueError('QR panels do not fit inside the competition field')

    positions = []
    min_distance = 2.0 * radius + 1.0
    for _ in TARGET_IDS:
        for _ in range(1000):
            x = center_x + random.uniform(-half_span, half_span)
            y = center_y + random.uniform(-half_span, half_span)
            if all(math.hypot(x - px, y - py) >= min_distance for px, py in positions):
                positions.append((x, y))
                break
        else:
            raise ValueError('Cannot place all QR panels with enough separation')
    return positions


def panel_sdf(target_id, size):
    return f'''<sdf version="1.6">
  <model name="{target_id}">
    <static>true</static>
    <link name="panel">
      <visual name="image">
        <pose>0 0 0 0 1.5707963267948966 0</pose>
        <geometry><mesh>
          <uri>model://tracking_target/meshes/panel.dae</uri>
          <scale>{size} {size} {size}</scale>
        </mesh></geometry>
        <material><script>
          <uri>model://competition_surface/materials/scripts</uri>
          <uri>model://competition_surface/materials/textures</uri>
          <name>Maxt/Competition/{target_id}</name>
        </script></material>
      </visual>
    </link>
  </model>
</sdf>'''


def main():
    rospy.init_node('competition_qr')
    panel_size = float(rospy.get_param('~panel_size'))
    positions = random_positions(
        rospy.get_param('~center_x'), rospy.get_param('~center_y'),
        rospy.get_param('~field_size'), panel_size)
    service = '/gazebo/spawn_sdf_model'
    rospy.wait_for_service(service, timeout=60.0)
    spawn = rospy.ServiceProxy(service, SpawnModel)
    for target_id, (x, y) in zip(TARGET_IDS, positions):
        yaw = random.uniform(-math.pi, math.pi)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = x, y, 0.01
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)
        result = spawn(target_id, panel_sdf(target_id, panel_size), '', pose, 'world')
        if not result.success:
            raise RuntimeError(f'Failed to spawn {target_id}: {result.status_message}')
        rospy.loginfo('Placed %s at (%.2f, %.2f), yaw %.1f deg', target_id, x, y, math.degrees(yaw))


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, ValueError, rospy.ROSException, rospy.ServiceException) as error:
        rospy.logerr('%s', error)
        sys.exit(1)
