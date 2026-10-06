#include <maxt_pkg/numeric_sim.hpp>
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "maxt_numeric_sim_node");
    // "~" 创建私有 NodeHandle，因此下方的 dt 实际读取
    // /maxt_numeric_sim_node/dt，不会与其他节点的同名参数冲突。
    ros::NodeHandle nh("~");

    // nh.param(name, output, default) 在参数缺失时保留 config 结构体中的默认值。
    maxt::NumericSimConfig config;
    nh.param("dt", config.dt, config.dt);
    nh.param("speed_factor", config.speed_factor, config.speed_factor);
    nh.param("report_period", config.report_period, config.report_period);
    nh.param("csv_path", config.csv_path, config.csv_path);
    nh.param("connected", config.connected, config.connected);
    nh.param("accept_arm", config.accept_arm, config.accept_arm);
    nh.param("accept_offboard", config.accept_offboard, config.accept_offboard);
    nh.param("report_landed", config.report_landed, config.report_landed);

    maxt::NumericSim sim(nh, config);
    if (!sim.ready()) {
        return 1;
    }

    // 单线程主循环：
    // 1) spinOnce() 先处理新 setpoint 和服务请求；
    // 2) step() 用最新命令推进固定 dt；
    // 3) WallDuration 特意使用真实时间休眠。如果换成 ros::Duration，
    //    在 /use_sim_time=true 时休眠本身反而会等待 /clock，形成自我等待。
    while (ros::ok()) {
        ros::spinOnce();
        sim.step();
        ros::WallDuration(sim.wallStepSeconds()).sleep();
    }
    return 0;
}
