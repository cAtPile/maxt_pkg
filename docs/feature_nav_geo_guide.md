# feature_nav_geo_guide

经纬度航向修改

## Supreme Principle
请阅读 SupremePrinciple.md

## 注意
不修改本文件

## 实现内容
1. 构建一个子树 NavGeoTree
2. 子树的 inport Longitude Latitude Hight
3. 关于 NavGeoTree 由 Geo2Local 和 NavRaw 组成
4. 构建一个投影转换节点 Geo2Local
5. 关于 Geo2Local 输入经纬度 输出该经纬度在 local 系下的 xy坐标
6. 将 Geo2Local 转换得到的 xy 经由 NavRaw 航向执行