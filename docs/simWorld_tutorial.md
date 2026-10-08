# Gazebo 场景使用与编写教程

本模块面向 ROS Noetic / Gazebo Classic 11。启动的是独立视觉测试环境；不包含无人机、PX4 SITL、MAVROS 或跟踪算法，不会自动执行飞行任务。

## 启动和图像检查

需要 Gazebo 开发包（`gazebo_dev`）、`gazebo_ros`、`gazebo_plugins`、`gazebo_msgs`、`rospy`、`tf2_ros`。在已有 Noetic 环境中缺少依赖时可安装：

```bash
sudo apt install ros-noetic-gazebo-ros-pkgs
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch maxt_pkg sim_world.launch
```

另开终端加载相同 ROS 环境后查看图像：

```bash
rosrun rqt_image_view rqt_image_view /sim_world/camera/image_raw
rostopic hz /sim_world/camera/image_raw
rostopic echo -n 1 /sim_world/camera/camera_info
```

GUI 中应看到地面、方箱、立柱、墙体和移动图片板。图像中可看到彩色几何图案与棋盘区，这是普通跟踪目标，不是二维码或 AprilTag。图像话题类型为 `sensor_msgs/Image`，标定话题为 `sensor_msgs/CameraInfo`。固定相机在世界坐标 (-2,2,6)，朝下；光学坐标为 X 向右、Y 向下、Z 向前，通过静态 TF 关联 `world`。

```bash
# 缩小运动范围并加快往返；参数单位分别为米和秒
roslaunch maxt_pkg sim_world.launch amplitude:=1.0 period:=8.0
# 图片保持 world 中定义的初始位置 (-2,2,0.02)
roslaunch maxt_pkg sim_world.launch move_target:=false
# 从暂停状态启动；在 Gazebo 中点击播放后开始运动
roslaunch maxt_pkg sim_world.launch paused:=true
```

`world` 可选其他 world 文件；`gui`、`paused` 控制 Gazebo；`move_target` 控制模型运动插件；`target_x/y/z` 控制移动中心，默认 (-2,2,0.02)；`amplitude` 默认 2m；`period` 默认 12s；`rate` 为兼容旧命令而保留，新插件忽略它。运动轨迹为 `x=x0, y=y0+A*sin(2*pi*t/T), z=z0`，按仿真时间运行，暂停会停止，重置仿真时间会重新开始相位。`period` 应大于零，`amplitude` 不小于零。模型内的 `libmaxt_moving_target_plugin.so` 在每个 Gazebo 物理步更新位置，不再逐帧调用 ROS 服务。参数在加载模型时读取，更改后需要重启。关闭移动时位置参数不生效。

相机水平视场约 60°，640×480、30Hz。`gui:=false` 仅关闭 Gazebo 界面，图像传感器仍需要可用的 OpenGL 渲染环境。若没有图像，检查 Gazebo 播放状态、相机插件是否加载以及显示/渲染环境。若图片为白色，检查 launch 中 `GAZEBO_MODEL_PATH` 和材质资源路径。

## 从零编写 world

world 是 SDF XML。最小结构如下；实际地面、光源和模型可参照 `worlds/tracking_test.world`：

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="my_world">
    <gravity>0 0 -9.81</gravity>
    <physics name="physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
  </world>
</sdf>
```

世界坐标 Z 向上；长度单位为米，旋转为弧度。`pose` 的六个值是 `x y z roll pitch yaw`。`model` 内的 link pose 相对模型，visual/collision pose 相对 link。先加光源、地面，再加障碍物和传感器，便于定位资源或渲染问题。本场景直接定义地面与光源，不需要下载在线模型。

静态障碍物由 `model/static`、`link`、`collision` 和 `visual` 组成。例如 1m 高方箱的中心 z 应为 0.5，使底面贴地：

```xml
<model name="my_box">
  <static>true</static>
  <pose>1 2 0.5 0 0 0</pose>
  <link name="body">
    <collision name="collision">
      <geometry><box><size>1 1 1</size></box></geometry>
    </collision>
    <visual name="visual">
      <geometry><box><size>1 1 1</size></box></geometry>
      <material><ambient>0.8 0.2 0.2 1</ambient><diffuse>0.8 0.2 0.2 1</diffuse></material>
    </visual>
  </link>
</model>
```

`visual` 决定外观，`collision` 决定碰撞，二者都要填写。圆柱使用 `cylinder/radius/length`；墙体可使用薄长方体。本场景三个 0.5×0.5×2m 方柱分别在 (2,2,1)、(4,1,1)、(4,3,1)，避开相机与图片板的默认视线。增加障碍物时为每个 model 使用不同名称，并检查目标运动范围是否穿过障碍物。

## 图片模型与材质

可复用模型放在 `models/<模型名>/` 下：`model.config` 描述元数据，`model.sdf` 描述模型，`meshes/` 放网格，`materials/scripts/` 放 OGRE 材质，`materials/textures/` 放图片。world 用以下方式引用：

```xml
<include>
  <uri>model://tracking_target</uri>
  <name>tracking_target</name>
  <pose>-2 2 0.02 0 1.57079632679 0</pose>
</include>
```

`GAZEBO_MODEL_PATH` 应指向 models 的父目录，本包 launch 已配置并保留已有路径。图片板使用带 UV 坐标的 DAE 网格；UV 指定图片如何铺到表面。板位于局部 YZ 平面、尺寸 1×1m，材质关闭背面剔除，因此两面均可见。碰撞体为厚 0.03m 的方板。

替换 `models/tracking_target/materials/textures/target.png` 即可换成自己的图片，重启 Gazebo 以重新加载纹理。若改文件名，同时改 `target.material` 中 `texture` 名称；若改模型目录名，同时修改所有 `model://` URI。图片比例不同会被映射到方板，需同步调整网格顶点和碰撞体尺寸以保持比例。

图片板使用 `static=true` 避免重力下落，由 Gazebo 模型插件在物理更新回调中直接设置位置。这适合视觉跟踪基准，不是物理驱动运动：碰撞不会阻止位置控制器把板移入障碍物。需要真实动力学时应改为动态模型，添加质量、惯性及关节/控制插件。

## 相机与后续无人机仿真

相机 sensor 写在 link 中，包含视场角、图像尺寸和裁剪范围；`libgazebo_ros_camera.so` 将数据发布到 ROS。默认相机局部 +X 经俯仰旋转后朝下看向图片板。改变相机 pose 时，必须同时更新 launch 中静态 TF；复制多个相机时也要分别设置模型名、sensor/plugin 名、ROS 命名空间和 frameName。

后续加入无人机时，把模型 spawn 到已启动的同一个 Gazebo 世界，并单独连接 PX4 SITL/MAVROS。不要再次启动另一个 gzserver。机载相机可复用此 sensor/plugin 配置并绑定到机体 link，使用与机体关联的光学 TF。现有航行和圆周任务仍要求有效的 MAVROS 状态与定位，单独启动本 world 不满足这些条件。

检查 SDF 格式可使用 `gz sdf -k <文件>`；运行后还应检查图片纹理、相机视野和运动轨迹，格式合法不等于视觉效果已验证。

参考：[Gazebo ROS 状态接口](https://classic.gazebosim.org/tutorials/?tut=ros_comm)、[官方 ROS 相机教程](https://github.com/osrf/gazebo_tutorials/blob/master/ros_gzplugins/tutorial.md)。

## 靶标运动卡顿修复

首次使用原生运动插件前重新编译本包并加载工作空间环境。若日志出现 `Failed to load plugin libmaxt_moving_target_plugin.so`，检查编译是否成功，以及终端是否已 source 对应工作空间的 devel/setup.bash 或 install/setup.bash。

两个入口 sim_world.launch 和 ref/sim.launch 都通过 /moving_target 命名空间提供 enabled、x、y、z、amplitude、period 参数；不再启动 Python 运动节点。现有 moving_target.py 仅作为旧方式保留，只有先使用 move_target:=false 禁用插件后才允许手动启动，避免两个控制器争抢位置。仿真暂停时不前进，重置世界或仿真时间时从轨迹中心重新开始。
