# 人工引导入口

人工悬停、Odin 画面打点和 AG_Client 路线编辑由 Catkin 包
`uav_manual_instruct` 提供。该包位于 `catkin_ws/src/uav_manual_instruct`，
使用独立 Git 仓库。通过 `~/overall.sh start start_manual_instruct:=true rviz:=false`
启动，浏览器访问 `http://127.0.0.1:8096/`。

人工引导模式和 `start_survey:=true` 互斥，同一架无人机只保留一个任务设定值
发布者。网页接管要求飞控报告已在空中，切换 OFFBOARD 由飞手执行。
