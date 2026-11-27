# feature_PolygonCoverage_guide

多边形遍历节点

## Supreme Principle
请阅读 SupremePrinciple.md

## 注意
不修改本文件

## 实现方法
1. 构建一个新的节点 PolygonCoverageWpG
2. 关于 PolygonCoverageWpG
    1. 本节点用来生成遍历的航点
    2. input 是四组经纬度 Longitude Latitude 工作高度 work_hight
    3. 这四个点可能是不按照顺序输入你要当心，始终保持其凸多边形边界
    4. 覆盖宽度可能除不尽,比方说范围是5,我设置宽度是2,如果按照宽度2的话最后会有1m没处理到
        我希望你用5/2向上取整到3,再用5/3得到实际的宽度
3. 构建 PolygonCoverageTree 子树
    1. input 四组经纬度和工作高度 
    2. 先使用 PolygonCoverageWpG 生成一系列航点
    3. 然后 抓取航点
    4. 最后 使用 NavAzimuth 执行航点