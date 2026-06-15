6-10
[19:31] RoundNode:改为前馈
        find：在很短距离内gotonode修正能力不足
        plan:GoToNode走若干小步，检查是否为setpoint固有缺点

[19:42] GoToNode:似乎微小步也可以较快到达
                再测试一下是否是偶然
                确实是偶然现象
        plan:测试RoundNode绕20圈

[19:51] RoundNode:前馈版本似乎没问题
        Thinking：末端对准精度不足是来源于setpoint在小步长的固有缺点，可能是pid的相应
                    我想到了3个方案：
                        1. 末端外加PID
                        2. 在最大速度改Pose控制
                        3. 真正闭环的五次曲线
        searching:      1. 加一个末端比例p
                        2. 位置速度，时间判断到达

[21:06] QuinticNav:位置速度判断到达失败
        Plan:末端P控制

[21:58] QuinticNav:末端加pi控制解决了问题，可能有些许抖动
        paln：降落锁定的优化

6-11
[10:26] RoundNode:不展示圈数，加速过大
        done:   1. （P1有严重风险）RoundNode改为5次曲线版本
                2. （P3低风险，优化调试体验）RoundNode增加圈数展示

[11:55] RoundNode:展示圈数
        plan:   1. 测试xml/launch
                2. 测试

6-12
[18:30] RoundNode:似乎没问题
        QuNav:似乎没问题，需要重新面向结果测试
        done:   2. （P3）forresult改写

[20:40] searching:/mavros/extended_state 可以判断接地状态
        plan：  1. 改一个全程方案
                2. ->试飞同时测试/mavros/extended_state话题
                3. ->检查QuNav是因为pid还是因为超时到达的

[21:09] find:   1. QuNav：存在超时场景
                2. /mavros/extended_state话题存疑
        doing： 改写TouchDown
        step:   1. mavkit添加 /mavros/extended_state 相关工具(v)
                2. 高度，垂直速度，/mavros/extended_state，联合判断(v)
                3. 超时强制上锁(v)
        done:   1. （P1）TouchDown降落锁定优化
        plan:写一个降落测试流程->测试降落，反馈
        caution: 由于室内定位特殊性，不使用Auto.Land

[22:17] TouchDown:3次均无弹跳，有待多次重复验证
        find：  悬停位置不固定会漂移
        done:   （P3）统一的测试launch
        plan:   建立统一的launch，进行实验和测试->清理xml和launch->建立waitstep的测试文件

6-13
[11:07] done:   1. （P2，影响任务效果）waitstep探索优化
                2. （P3）清理launch，xml
                3. （P4，锦上添花）统一日志格式
                4. （P4）改QuNav改debug模式
        find:   1. 全程测试发现wait问题并非严重，问题暂时搁置
                2. 低速下降有概率无法锁定
        plan:做下午试飞的内容：
                1.绕圆一周
                2.仅投放
                3.面向结果

[17:16] talk:确实没啥能改的

6-14
[0:52]  find:   1. 圆环速度突变
                2. 掉落弹跳问题
        Aing：  tick没赋值
        plan：  仿真测试改进版—>试飞找问题

[12:23] RoundNode:仿真没问题了，等试飞
        talk：硬找不出什么问题了，边飞边改吧，剩下的省赛后再说吧
        TODO：  1. （P2）建立一个方便catkin的git文件
                2. （P2）创建hover代替wait
        plan：建立git仓库->转移—>发布

[15:56] git:现在切换到了新的工作空间
        find：任然有降落弹跳
        thinking：应该是降落为了确保位置精准的调整，可以在低空只设置速度不保持位置吗，可以只看触地话题吗

[16:31] doing:创建工程专用skill

[16:48] doing:created HoverNode
        TODO:   1.(P1)modify TouchDown
        plan: sim test HoverNode->creat xml->test

[21:09] find:坐标系问题，每次启动都会有偏移
        thinking：      1. 坐标相对化 
                        2. 自动测绘脚本
                        3. 微小的误差累计的大量的线性偏差
                        4. 起飞前偏航归零
                        5. 四点标定是否可以？
        soultion:       1. 确保每次起飞位置完全相同
                        2. 标定 
        plan:   creat calibrate_tools:  1. input:setmap(mapx,mapy)，这是我地图上的坐标
                                        2. input:setfact(factx,facty),这是标定位置的坐标
                                        3. transRaw/transPose,将输入的转化为标定后的
                modify mav_kit: 构造函数创建CalibrateTools全局实例
                                增加标定函数mavCalibrate，input:使用setmap/fact
                                raw和pose经过转换后再发布（心跳线程里处理）
                                getpose/twist经过一步处理后返回
                modify bt：在connect中添加标定步

[23:02] calibrate:似乎发挥着作用
        find：距离缩放，但经过计算，距离缩放影响似乎不大，但一定要考虑
        plan：试飞看看效果