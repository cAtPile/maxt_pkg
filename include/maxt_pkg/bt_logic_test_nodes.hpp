#ifndef MAXT_PKG_BT_LOGIC_TEST_NODES_HPP
#define MAXT_PKG_BT_LOGIC_TEST_NODES_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/condition_node.h>

#include <string>

namespace maxt {

/**
 * @brief 返回配置的布尔结果，用于验证条件节点和 IfThenElse 分支。
 */
class TestConditionNode : public BT::ConditionNode {
public:
    TestConditionNode(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

/**
 * @brief 前若干次返回 SUCCESS，随后返回 FAILURE，用于验证响应式分支切换。
 */
class TestTickConditionNode : public BT::ConditionNode {
public:
    TestTickConditionNode(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    unsigned int tick_count_{0};
};

/**
 * @brief 输出一条测试日志并立即成功，不执行任何外部命令。
 */
class TestLogNode : public BT::SyncActionNode {
public:
    TestLogNode(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

/**
 * @brief 保持 RUNNING 指定 tick 数，用于观察 WhileDoElse 的 halt 行为。
 */
class TestRunningNode : public BT::StatefulActionNode {
public:
    TestRunningNode(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::string label_;
    unsigned int running_ticks_{1};
    unsigned int tick_count_{0};
};

/**
 * @brief 前 N 次执行失败、之后成功，用于验证 RetryUntilSuccessful。
 */
class TestFlakyNode : public BT::SyncActionNode {
public:
    TestFlakyNode(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    unsigned int attempt_count_{0};
};

}  // namespace maxt

#endif  // MAXT_PKG_BT_LOGIC_TEST_NODES_HPP
