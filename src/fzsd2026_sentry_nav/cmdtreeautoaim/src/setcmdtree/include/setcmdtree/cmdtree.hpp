// cmdtree.hpp - 决策树函数声明
#ifndef CMDTREE_HPP
#define CMDTREE_HPP

#include <cstdint>
#include <string>

// 决策树输入结构体
struct CmdTreeInput {
    int current_robot_quantity;      // 当前存活机器人数量 (1-3)
    int blood_warn_state;            // 血量警告状态值 (0-100)
    int game_process;                // 比赛进程状态 (4=比赛进行中)
    float current_state_left_time;   // 当前阶段剩余时间（秒）
    uint8_t is_success_ourpreempt;   // 我方占点成功标志 (0/1)
    uint8_t is_success_enemypreempt; // 敌方占点成功标志 (0/1)
    int8_t is_ALL_state_level;       // 整体战力评估 (-1/0/1)
    uint8_t gyroscope;
    double odom_x;                   // 里程计 x 坐标（来自 odometry 话题）
    double odom_y;                   // 里程计 y 坐标（来自 odometry 话题）
};

// 决策树输出结构体
struct CmdTreeOutput {
    uint8_t robot_state_level;       // 机器人状态等级 (1-6)
    int vision_nav_cmd;              // 导航命令 (0-4)
    int fire_decision;               // 打击决策 (123/213)
    uint8_t gyroscope;               // 陀螺仪/姿态决策结果 (0-4，来自 GyroscopeDecision)
    std::string description;         // 决策描述
};

// 主决策树函数（新版本：接收输入，返回输出）
CmdTreeOutput CmdDecisionTree(const CmdTreeInput &input);

#endif // CMDTREE_HPP
