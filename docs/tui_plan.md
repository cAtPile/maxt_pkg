# tui_plan

建立一个tui的工作台可以进行操作

## 需求
1. 启动
2. 状态监看，复杂状态查看
3. 复合任务模板编排

## draft
1. 先做一个结构化的xml生成器
2. 启动后我可以选择
```
mission list：
1. takeoff
2. nav
....
```
我通过键盘选择
3. 选择后我可以输入具体的参数，
```
takeoff altitude:
```
4. 最后生成
```
<root main_tree_to_execute=" ">
    <BehaviorTree ID=" ">
        <Sequence>
            <ConnectCheck timeout="30.0" />
                连接检查和落地不改变
                中间是我输入的标准化的btxml
            <TouchDown descent_speed="-0.3" timeout="60.0" ground_timeout="3.0" />
        </Sequence>
    </BehaviorTree>
</root>
```
到/mission/fast_mission.xml
5. 新生成的覆盖原本的/mission/fast_mission.xml