# guidanceNode_guide

构建引导模式的行为树节点

## Supreme Principle
请阅读 SupremePrinciple.md

## 实现内容
1. 在heartbeatTimerCallback增加一个模式GUIDANCE
2. 定义一个action
    ```
    goal:
        string target_id        #物品id
    feedback:    
        bool target_visible     #是否可视
        float32 error_u         #偏差
        float32 error_v      
    result：
        string message          #结果 SUCCEED FAIL TIMEOUT ...
    ```
3. 端口获取需要跟踪的 target_id,bias_u/v,tolerance,timeout
4. onstart阶段：订阅这个action，发布goal
5. onrunning阶段：  状态1 :等待第一个feedback，
                    状态2 ：得到第一个feedback后，将得到的feedback后经过PID处理后将结果填充到target，切换为RWA_CTRL模式并持续检查error是否满足tolerance
                    状态3：error是否满足tolerance后，切换回HEARTBEAT

## 注意
不修改本文件