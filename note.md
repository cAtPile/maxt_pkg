```
rosservice call /mavros/cmd/arming "value: true"
```

```
rostopic echo /mavros/global_position/global
```
```
rostopic pub -r 20 /QRCode_pose/Global_pose \
sensor_msgs/NavSatFix \
"header:
  stamp: now
  frame_id: 'gps'
latitude: 47.397832
longitude: 8.545594
altitude: 5.0"
```

```
catkin_make -j3 -DMAXT_ENABLE_QR_SURVEY=OFF \
  --only-pkg-with-deps maxt_pkg slicer_air_ground_msgs
source devel/setup.bash
```