# uav_audio 接入说明

版本：0.1.0；更新日期：2026-10-27。

## 1. 功能与运行位置

`uav_audio` 是 ROS1 Noetic C++14 包，通过 `/audio/play` 服务异步启动 ALSA `aplay`，播放本包 `audio/` 目录中的 WAV。一次只播放一条，忙时直接拒绝，无队列。

节点运行在哪台主机，声音就从那台主机的音频设备输出。客户端可以是同一 ROS 网络中的其他节点。已在 Ubuntu 20.04 / Jetson Orin NX / ROS Noetic 上编译及实机验证。

源码发布目标：`https://github.com/jiazhuguanhua/uav_audio`。

## 2. 安装与编译

已安装 ROS Noetic 的 Ubuntu 20.04 主机：

```bash
sudo apt-get install build-essential alsa-utils ros-noetic-catkin \
  ros-noetic-roscpp ros-noetic-roslib \
  ros-noetic-message-generation ros-noetic-message-runtime
```

仓库发布后，将它作为一个包克隆到自己的 Catkin 工作空间 `src/` 下：

```bash
source /opt/ros/noetic/setup.bash
mkdir -p ~/audio_ws/src
cd ~/audio_ws/src
git clone https://github.com/jiazhuguanhua/uav_audio.git
cd ~/audio_ws
catkin_make -j4
source devel/setup.bash
```

也可以将包放进现有工作空间，无需创建 `audio_ws`。当前开发机在 `~/devices_ws`，含 Livox 的整个工作空间构建命令为：

```bash
cd ~/devices_ws
source /opt/ros/noetic/setup.bash
catkin_make -DROS_EDITION=ROS1 -j4
source devel/setup.bash
```

每个新终端都需要加载 ROS 和自己的工作空间环境。例如当前开发机：

```bash
source /opt/ros/noetic/setup.bash
source ~/devices_ws/devel/setup.bash
```

`setup_devices.bash` 是当前开发机已有的便捷脚本，等价于加载上述两层环境；同事的机器无需这个脚本。

## 3. 选择声卡与启动

```bash
aplay -l   # 查看硬件声卡
aplay -L   # 查看 ALSA 逻辑设备名
```

节点名为 `/audio_node`；私有参数 `~audio_device` 是字符串，默认 `default`，由 launch 的 `audio_device` 参数传入。

当前开发机的 USB 喇叭通过 PulseAudio 输出，启动方式：

```bash
roslaunch uav_audio audio.launch audio_device:=pulse
```

这里仍由 `aplay` 播放，`pulse` 是 ALSA 的 PulseAudio 输出插件。需要运行用户的 PulseAudio 会话并正确选择默认输出设备；Ubuntu 若缺少插件，可安装 `libasound2-plugins`。当前开发机 `/etc/asound.conf` 的 ALSA `default` 指向 Jetson 内置 APE 声卡，与桌面选择的 USB 默认输出不同。

其他机器若已正确配置 ALSA 默认输出：

```bash
roslaunch uav_audio audio.launch
```

也可以指定 USB 硬件名（以 `aplay -l` 的结果为准）：

```bash
roslaunch uav_audio audio.launch audio_device:=plughw:CARD=Device,DEV=0
```

直接访问硬件时，声卡不能被其他程序独占。音频设备也可先脱离 ROS 单独验证：

```bash
aplay -D pulse "$(rospack find uav_audio)/audio/cn-ready.wav"
```

## 4. 服务接口与命令行调用

服务名固定为 `/audio/play`，类型为 `uav_audio/PlayAudio`：

```text
string filename
---
bool success
string message
```

启动节点后，另一个已经加载工作空间环境的终端执行：

```bash
rosservice type /audio/play
rossrv show uav_audio/PlayAudio
rosservice call /audio/play "filename: 'cn-ready.wav'"
```

正常启动播放时立即返回：

```yaml
success: true
message: "Playback started"
```

播放中再次请求：

```yaml
success: false
message: "Audio device busy"
```

`success=true` 只表示播放进程成功启动，不代表设备成功打开、实际发声或播放完成。若 `aplay` 后续发现设备不可用或 WAV 损坏，会记录错误并恢复空闲。客户端需要同时判断 ROS 调用是否成功以及响应中的 `success`，业务拒绝不会作为 ROS 传输错误返回。

| 情况 | success | message |
| --- | --- | --- |
| 已启动进程 | true | `Playback started` |
| 当前播放中 | false | `Audio device busy` |
| 非法文件名 | false | `Invalid filename: ...` |
| 文件缺失或不可读 | false | `Cannot open WAV file '...': ...` |
| 路径是目录或 FIFO 等 | false | `WAV file is not a regular file` |
| 音频目录不可访问 | false | `Cannot open audio directory: ...` |
| 无法启动 aplay | false | `Cannot start aplay: ...` |

文件名只允许纯文件名，扩展名必须是小写 `.wav`；不能含 `/`、反斜线、`..` 或 NUL。不接受绝对路径、子目录和符号链接。不经过 Shell 执行文件名。

## 5. C++ 客户端接入

在调用方包的 `package.xml` 增加：

```xml
<depend>roscpp</depend>
<depend>uav_audio</depend>
```

把 `roscpp` 和 `uav_audio` 合并进调用方原有的 `find_package(catkin REQUIRED COMPONENTS ...)`。一个最简独立示例的 CMake 配置如下，目标名及源码路径可按实际项目调整：

```cmake
find_package(catkin REQUIRED COMPONENTS roscpp uav_audio)
catkin_package(CATKIN_DEPENDS roscpp uav_audio)
include_directories(${catkin_INCLUDE_DIRS})
add_executable(audio_client src/audio_client.cpp)
target_link_libraries(audio_client ${catkin_LIBRARIES})
add_dependencies(audio_client ${catkin_EXPORTED_TARGETS})
```

`src/audio_client.cpp`：

```cpp
#include <ros/ros.h>
#include <uav_audio/PlayAudio.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "audio_client");
  ros::NodeHandle nh;
  auto client = nh.serviceClient<uav_audio::PlayAudio>("/audio/play");
  if (!client.waitForExistence(ros::Duration(3.0))) {
    ROS_ERROR("/audio/play unavailable");
    return 1;
  }

  uav_audio::PlayAudio srv;
  srv.request.filename = "cn-ready.wav";
  if (!client.call(srv)) {
    ROS_ERROR("ROS service call failed");
    return 1;
  }
  if (!srv.response.success) {
    ROS_WARN_STREAM("Playback rejected: " << srv.response.message);
    return 2;
  }
  ROS_INFO_STREAM(srv.response.message);
  return 0;
}
```

业务节点在需要播报的事件处调用服务即可。同步的 ROS 服务调用只等待启动结果，不等待整条音频结束。忙时由业务侧决定忽略本次播报或稍后再调用；服务端没有队列、Stop 或 Status 接口。

## 6. 已有音频

原始样例：`sample-6s.wav`（约 6.39 秒）。中文提示音为 PCM WAV / 24 kHz / 16 bit / 单声道，文案如下：

| 文件名 | 文案 |
| --- | --- |
| `cn-ready.wav` | 飞行系统已就绪。 |
| `cn-armed.wav` | 飞机已解锁，请远离飞行器！ |
| `cn-takeoff.wav` | 飞机即将起飞，请保持安全距离！ |
| `cn-mission-start.wav` | 开始执行任务。 |
| `cn-target-found.wav` | 发现目标，开始执行任务。 |
| `cn-target-lost.wav` | 目标丢失，正在重新搜索。 |
| `cn-mission-complete.wav` | 任务执行完成。 |
| `cn-return.wav` | 飞机正在返航，请保持降落区域畅通。 |
| `cn-landing.wav` | 飞机即将降落，请远离降落区域！ |
| `cn-disarmed.wav` | 飞机已上锁。 |
| `cn-battery-low.wav` | 电量偏低，请尽快降落。 |
| `cn-battery-critical.wav` | 电量严重不足，立即降落！ |
| `cn-fault.wav` | 出现一般故障，请立即降落！ |
| `cn-fault-critical.wav` | 出现严重故障，立即降落！ |
| `cn-welcome.wav` | 欢迎使用段家威飞行器。 |

新增 WAV 时复制到 `$(rospack find uav_audio)/audio/`，然后按文件名请求。使用源码工作空间时，仅新增音频不需要重新编译；若部署使用 install 空间，还需将新增文件安装或同步到该空间的包内音频目录。

## 7. 多机调用

服务端和客户端要连接同一个 ROS Master，并配置可互相访问的 `ROS_MASTER_URI` 及 `ROS_IP` / `ROS_HOSTNAME`。客户端需安装并编译 `uav_audio`，以获得服务类型；扬声器和 WAV 实际由服务端主机使用。可先检查：

```bash
rosnode list
rosservice list
rosservice info /audio/play
```

## 8. 验证范围与常见问题

已完成整个 `devices_ws` 编译、37 项功能及鲁棒性检查，另对 15 条中文音频逐条做了真实服务播放测试：每条 `aplay` 退出码均为 0；服务响应约 5–7 ms；播放期间重复请求返回 busy，结束后可继续播放。覆盖并发请求、非法文件名、符号链接、不可读文件、FIFO、缺失 aplay、重复失败资源检查及节点退出清理。

验证依据为音频输出路由及进程退出状态，未通过麦克风测量实际声压。本文客户端示例供调用方集成参考，尚未作为独立客户端包编译测试。

- 找不到包或服务类型：先编译，再在当前终端 `source <工作空间>/devel/setup.bash`。
- 服务不存在：确认服务端 launch 已运行，并确认双方 ROS Master 一致。
- 返回 busy：当前音频尚未结束；服务端不排队。
- 返回 true 但无声：查看节点终端中的 ALSA 错误；检查 `audio_device`、音量、静音和实际扬声器连接，再用 `aplay -D <设备> <WAV路径>` 单独测试。
- 退出日志显示 aplay 非零返回：属于启动后的播放错误；服务会恢复空闲。
- Ctrl+C 关闭节点时，节点会终止并回收其播放子进程。
