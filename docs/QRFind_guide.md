QRFind_guide.md

区域遍历，查找二维码

## Supreme Principle
请阅读 SupremePrinciple.md

## 注意
不修改本文件

## 实现内容
1. 新增一个节点FoundCheckNode用来检查是否找到目标
2. 实现的结果如下，边飞行边检测二维码，检测到二维码后停止遍历
    参考行为树如下

```
<Sequence>
    <!-- 计算航点，初始化索引，写入黑板 -->
    <RegionTraverse wp_num="{wp_num}"/>

    <ReactiveFallback>
        <!-- 找到返回 SUCCESS，未找到返回 FAILURE -->
        <FoundCheck/>

        <!-- 未找到时，执行整个航点遍历 -->
        <Repeat num_cycles="{wp_num}">
            <Sequence>
                <GetWp goal="{current_goal}"/>
                <Nav goal="{current_goal}"/>
                <!-- 到达后推进索引，或由 GetWp 内部维护索引 -->
            </Sequence>
        </Repeat>
    </ReactiveFallback>

    <!-- 找到目标，或全部航点遍历完成后，执行这里 -->
    <NextTask/>
</Sequence>
```