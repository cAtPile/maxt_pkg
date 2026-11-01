# audio_guide

语言播放模块添加

## Supreme Principle
请阅读 SupremePrinciple.md

## 注意
不修改本文件

## 实现方式
1. 启动程序后播放：cn-welcome.wav
2. 连接到飞控后播放：cn-ready.wav
3. 解锁后播放：cn-armed.wav
4. 起飞开始播放：cn-takeoff.wav
5. 起飞过程中播放：sample-6s.wav
6. 新增一个行为树节点播放语音 输入为播放的内容 这是一次触发的内容不要阻塞任务 
7. 开始降落时播放 cn-landing.wav
8. 上锁后播放：cn-disarmed.wav

## 修改意见
1. 降落过程播放：cn-welcome-sua.wav
2. 解锁和上锁均在行为树的起飞或降落节点播放，不在stateCb中播放
3. stateCb中不做任何播放操作，连接的提示在行为树的connectcheck中播放