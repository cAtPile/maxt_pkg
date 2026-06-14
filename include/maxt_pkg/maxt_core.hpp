/**
 * @file maxt_core.hpp
 * @brief MaxtCore 类的头文件
 * @date 4-14
 */
#ifndef MAXT_CORE_HPP
#define MAXT_CORE_HPP

#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>

#include <maxt_pkg/maxt_mav.hpp>

namespace maxt {

/**
 * @class MaxtCore
 * @brief MaxtCore: 系统的调度核心
 * 负责：初始化组件、注册节点、加载行为树、维持主循环
 */
class MaxtCore {
public:
    /**
     * @fn MaxtCore
     * @brief 构造函数
     * @param nh ROS节点句柄
     */
    MaxtCore(ros::NodeHandle& nh);

    /**
     * @fn ~MaxtCore
     * @brief 析构函数，负责安全关闭系统
     */
    ~MaxtCore();

    /**
     * @fn init
     * @brief 系统初始化
     * @details 包括：加载参数、初始化 MavKit、注册 BT 节点、加载 XML
     * @return true 初始化成功
     */
    bool init();

    /**
     * @brief 启动行为树主循环
     * 内部包含 tick 频率控制和 AsyncSpinner 调度
     */
    void run();

private:
    /**
     * @brief 注册自定义行为树节点
     * 将 MavKit 实例通过依赖注入方式提供给各个节点
     */
    void registerNodes();

    // ROS 相关
    ros::NodeHandle nh_;
    // 关键：异步 Spinner，确保回调在后台线程独立运行，不阻塞 BT
    std::unique_ptr<ros::AsyncSpinner> spinner_;

    // 硬件抽象层
    std::unique_ptr<MavKit> mav_;

    // 投放机构 publisher — 系统初始化时提前建立，避免 BT 节点临时建 pub 导致延迟丢消息
    ros::Publisher front_left_open_pub_,  front_left_close_pub_;
    ros::Publisher front_right_open_pub_, front_right_close_pub_;
    ros::Publisher back_left_open_pub_,   back_left_close_pub_;
    ros::Publisher back_right_open_pub_,  back_right_close_pub_;
    ros::Publisher all_open_pub_, all_close_pub_;

    // BehaviorTree 相关
    BT::BehaviorTreeFactory factory_;
    BT::Tree tree_;
    
    // 日志与调试
    std::unique_ptr<BT::StdCoutLogger> logger_;

    // 配置参数
    std::string xml_path_;
    double tick_rate_{20.0}; // 行为树运行频率（Hz）
};

} // namespace maxt

#endif // MAXT_CORE_HPP