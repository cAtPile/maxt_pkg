# feature_Boustrophedon_guide

本模块用来进行矩形牛耕遍历

## Supreme Principle
请阅读 SupremePrinciple.md

## 注意
不修改本文件

## 实现内容
1. 修改 TraversePathGeneratorNode 节点
    关于 TraversePathGeneratorNode 节点：
    1. input：  origin_x/y 原点的local坐标
                lenth_n/e 北向和东向的边长
                coverage_step 覆盖范围
                heading_yaw 矫正航向
    2. output:  waypoints 类型：'RegionWaypoints' 航点集 local 坐标系
                waypoint_count 航点数量

2. 生成一个 BoustrophedonTree 子树 ，本子树用来进行区域牛耕遍
    关于BoustrophedonTree 子树：
    1. input：  base_Longitude/Latitude 基准点的经纬度
                lenth_n/e 北向和东向的边长
                coverage_step 覆盖范围
    2. 首先使用 Geo2Local 将 base_Longitude/Latitude 转换为 local 系作为 TraversePathGeneratorNode 的 origin_x/y，
        然后调用 TraversePathGeneratorNode 生成航点
        然后构建一个有限循环 使用NavRaw巡航
