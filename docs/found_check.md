# FoundCheck 内容去重

`FoundCheck` 订阅 `std_msgs/String`，默认话题为 `/aruco_center/content`。空字符串表示未发现目标；消息超时仍由 `~found_check/timeout` 控制。

首次 tick 检测到有效且未记录的内容时，立即将其加入任务根黑板 `processed_qr`（`std::vector<std::string>`），返回 `SUCCESS`。重复内容、空内容和超时返回 `FAILURE`，让遍历继续。即使随后跟踪失败，已记录的内容也不会自动删除。

`MaxtCore` 向各个 FoundCheck 实例注入同一个任务根黑板，因此记录跨子树、跨航段共享。记录保存在内存中，重启程序后不保留。

```xml
<FoundCheck topic="/aruco_center/content" />
```

`record="false"` 只检查目标是否可见，不查询或修改数组，供原有的目标消失检查使用。

当前视觉节点识别 ArUco，发布内容为 ID 字符串（例如 `"0"`），未发现时发布空字符串。若更换为普通二维码解码器，向同一话题发布解码文本即可。不同实体使用相同内容时，会被视为同一个目标。

自定义代码直接构造 FoundCheck 时，可通过构造函数的 `history` 参数传入共享黑板；未传入则使用节点自己的黑板。共享黑板中的记录可以通过 `get<std::vector<std::string>>("processed_qr")` 读取。
