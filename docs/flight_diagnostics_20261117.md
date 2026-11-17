# 2026-11-17 QR survey diagnosis

Times below are Asia/Shanghai. Reviewed ROS session:
`/home/dlab/.ros/log/fccd6654-e24f-11f1-b1ac-3c6d662cd815`.
No aircraft mission, arming, takeoff, mode changes or hardware configuration changes were performed during diagnosis.

## Flight result and evidence

- 14:30:24 externally armed; 14:30:25 OFFBOARD confirmed. Survey origin (-0.017,-0.020), heading -0.78 degrees, target local Z 6.02 m.
- 14:30:48–14:32:55: all 16 stops completed fresh captures. The configured rectangle (15,-15), step 5 produces 4 x 4 stops. The YAML's existing “3 x 3 = 9” comment is stale.
- 14:32:55: `QrFinish: drop code has no valid measured position; holding instead of navigating.` Immediately followed by `SurveyFailureHold`. No `ToDropTarget` navigation was started. See rosout.log:810–813.
- 14:33:07: Odin SDK reported `Device detaching...`, stopped its IMU thread, then waited for reconnection. See rosout.log:829–833. This happened AFTER completion of the grid and rejection of target navigation; it does not explain the earlier missing QR001 position.
- 14:33:53: FCU reported `Kill engaged` and `Flight termination active` (rosout.log:882 onward). Logs alone do not establish operator intent.
- 14:36:00: launch was stopped with SIGINT. Odin's detach was not a process exit at that time; the driver remained alive until launch shutdown.

`CAPTURED` means an accepted fresh image/detection/transform pair was processed. Empty detections can still produce CAPTURED. The previous logs do not archive decoded IDs, per-target validity/reasons, or source images. They establish QR001 lacked an eligible position, but cannot distinguish no decoded QR001 from failed depth/geolocation. No claim that all QR codes were unseen can be made from these logs.

## Odin connection and newly attached speaker

Read-only kernel journal observations:

- 14:24:55: USB audio device attached at `1-1` (8087:1024).
- 14:29:48: Odin at `1-3.3` disconnected.
- 14:29:51–53: Odin re-enumerated as **high-speed USB**, with `device descriptor read/64, error -71` during enumeration. High-speed is USB 2.0, 480 Mb/s; a physical USB 3 connector/cable does not guarantee SuperSpeed negotiation.
- No USB disconnect/reset, overcurrent, OOM or host undervoltage message was found at the 14:33:07 SDK detach. Its low-level reason is not exposed in rosout.

Read-only sysfs/lsusb topology at diagnosis time:

```text
Jetson xHCI
  USB2 root
    1-1    USB audio, 12 Mb/s
    1-2    CubeOrange+, 12 Mb/s
    1-3    USB2.0 HUB (1a40:0101), 480 Mb/s
      1-3.2  UVC camera (05a3:9230), 480 Mb/s
      1-3.3  Odin / hawk (2207:0019), 480 Mb/s
      1-3.4  Bluetooth, 12 Mb/s
  USB3 root, 10000 Mb/s: no devices shown
```

Odin shares its USB2 hub's upstream bandwidth with the UVC camera. The speaker is on another root port, so it is not directly competing for that hub's upstream bandwidth. Common upstream power/controller effects remain possible; this topology does not prove the speaker caused the failure. Descriptor bMaxPower values describe requested power, not actual consumption or voltage measurements.

Device status CSV:
`/home/dlab/devices_ws/src/odin_ros_driver/log/Driver_20261117_143006/Conn_20261117_143006/dev_status.csv`.
101 samples; last uptime 510.02 s. Package 24–31 C, CPU 23–32 C, DTOF RX 37 C; device RAM 4–17%, driver RAM 138.50–258.86 MB. Last RGB/DTOF receive rates about 9.65/9.72 Hz. No clear thermal or runaway memory signature in this record. RGB configuration command failed at 14:30:11 (`rc=-1`), so requested settings should not be treated as confirmed; frames nevertheless arrived throughout all captures.

Ground follow-up: verify the actual Odin route with `lsusb -t` after connecting it to the intended USB3 path. The current 1a40:0101 hub advertises USB2 only. Compare stationary streaming with the speaker disconnected/connected, then with playback active; keep all other connections fixed. This diagnosis did not unplug devices, reset Odin or change power/USB settings.

## RViz and other observations

`qr_rviz_markers.py` subscribes to `/uav_odin_qr/tf_targets`, not the frozen mission result. Each valid depth target is transformed into `uav/px4_local`. Each frame first clears previous markers, and marker lifetime is only 0.7 s. Capture-only survey images arrive roughly every 8 seconds, so QR markers can flash and disappear between stops. Missing QR markers alone do not establish failed decoding. No `QR RViz transform unavailable` warning was found in this flight's rosout. Marker behavior was left unchanged.

Kernel logs also show repeated WLAN losses starting 14:31:05, including 14:31:18, 14:32:18 and 14:33:01. This can interrupt remote monitoring; onboard captures continued across those intervals. It is distinct from the Odin USB connection.

BT already has StdCoutLogger transitions and `/maxt_test_node/mission_status` snapshots. A persistent hold stays RUNNING; absence of `Mission Failed!` does not imply success. The decisive line is the failed action followed by the named hold. Existing BT transition format:

```text
[<ROS seconds>]: QrFinish                  RUNNING -> FAILURE
[<ROS seconds>]: SurveyFailureHold         IDLE -> RUNNING
```

These are format examples, not recovered stdout from this flight. StdCoutLogger writes terminal stdout; diagnostic ROS_INFO/ERROR messages also reach rosout.log.

## Minimal diagnostic changes

Only diagnostic output was changed in two source files. No mission decisions, navigation parameters, decoding, depth, geolocation, or RViz lifetime behavior were changed. Existing uncommitted YAML/XML edits were preserved and excluded from diagnostic commits.

- `uav_qr_mission/node.py`: print current accepted frame's decoded contents/depth statuses, then cumulative registered-target states at capture completion.
- `maxt_pkg/qr_finish.cpp`: report registered-target found/position/revision/reason, and explain every existing finish failure path. Retain the existing successful target coordinate log.

Example formats (illustrative, not this flight's recovered recognition results):

```text
QR frame capture_id=<id> image_stamp=<ns> decoded=0 depth_targets=0 observations=[]
QR capture <id>: CAPTURED, image_stamp=<ns>
QR survey target=QR001 seen_in_survey=False position_valid=False reason=WAIT_OBSERVATION

QR frame capture_id=<id> image_stamp=<ns> decoded=1 depth_targets=0 observations=[('QR001', False, '<depth status>')]
QR survey target=QR001 seen_in_survey=True position_valid=False reason=NO_UNIQUE_VALID_DEPTH

QrFinish: target=QR001 seen_in_survey=1 position_valid=0 revision=0 reason=NO_UNIQUE_VALID_DEPTH
QrFinish: navigation blocked: target=QR001 has no valid measured position; geolocation_input=<status>; entering SurveyFailureHold. See target seen/position/reason above.

QrFinish: full grid completed; frozen drop target QR001 at <latitude>, <longitude>; all_localized=<0|1>
[NavGeo] ...
```

`QR frame` describes the current accepted capture frame; `seen_in_survey` is cumulative and never means “visible right now”. A decoded ID can be outside the registered list; observations preserve it for diagnosis. `position_valid` indicates the mission's retained geolocation, not just an image pixel. Reasons such as WAIT_OBSERVATION, NO_UNIQUE_VALID_DEPTH, OBSERVATION_EXPIRED and geolocation status identify different stages.

## Validation

- Existing mission/geolocation/clock unit tests: 30 passed.
- Existing stationary-capture ROS contract: 3 passed on independent loopback master `127.0.0.1:11587`, with only roscore and uav_qr_mission; no MAVROS, BT, hardware driver or flight commands. Covers valid localization, cancellation, stale depth, and missing-frame timeout. Diagnostic output verified in rosout. Test logs: `/tmp/qr-diagnostics-20261117/`.
- C++ build: `catkin_make --pkg maxt_pkg -j2 -l2` passed, including maxt_test_node. No BT was executed.
