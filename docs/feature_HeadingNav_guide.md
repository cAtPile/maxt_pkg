# feature_HeadingNav_guide

本模块用来进行固定航向的航行

## Supreme Principle
请阅读 SupremePrinciple.md

## 注意
不修改本文件

## 实现内容
1. 建立一个坐标转换节点 Heading2Local 将固定航向的坐标转换为本地坐标系，然后使用 NavRaw 进行航行
    关于 Heading2Local 节点:
    1. input:   headingYaw 航向角度 target_t/n 切向与法向位移
                base_Longitude/Latitude 基准点的经纬度
    2. output:  local系的x/y 供 NavRaw 使用
2. 建立一个高度转换的节点 HightRef2Local 将输入的高度相对home点的高度处理
    关于 HightRef2Local 节点:
    1. input:   起飞时发出的 home_alt={return_z}  target_hight 目标高度
    2. output:  local系的z 供 NavRaw 使用
3. 建立一个子树 HeadingNavTree 用来进行固定航向的航行
    关于 HeadingNavTree 子树
    1. input:   base_Longitude/Latitude 基准点的经纬度，供 Heading2Local 使用
                headingYaw 航向角度 target_t/n 切向与法向位移，供 Heading2Local 使用
                ？{return_z}是否需要显示写入