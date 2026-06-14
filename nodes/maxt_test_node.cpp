#include <ros/ros.h>
#include <maxt_pkg/maxt_core.hpp>

int main(int argc, char** argv)
{
    // 1. 初始化 ROS 节点
    ros::init(argc, argv, "maxt_test_node");
    ros::NodeHandle nh("~"); // 使用私有句柄以方便读取参数

    ROS_INFO("========================================");
    ROS_INFO("    Maxt Drone BehaviorTree System      ");
    ROS_INFO("========================================");

    // 2. 实例化系统核心
    // MaxtCore 内部会自动管理 MavKit 和 BT 节点的注册
    maxt::MaxtCore core(nh);
    
    // 3. 初始化系统
    if (!core.init()) {
        ROS_FATAL("Failed to initialize MaxtCore. Shutting down...");
        return -1;
    }

    // 4. 进入运行循环
    // 该函数是阻塞的，直到行为树运行完成（SUCCESS/FAILURE）或 ROS 关闭
    core.run();

    ROS_INFO("Maxt Mission Completed. Exiting...");
    
    return 0;
}