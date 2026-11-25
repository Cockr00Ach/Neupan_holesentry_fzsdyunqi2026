#include "setcmdtree/cmdtree.hpp"
#include <cstdint>
#include <ctime>
#include <cstdio>
#include <string>
#include <sstream>

// 血量阈值常量定义
#ifndef ROBOT_BLOOD_PERFECT
#define ROBOT_BLOOD_PERFECT 90  // 完美血量阈值
#endif
#ifndef ROBOT_BLOOD_HIGH
#define ROBOT_BLOOD_HIGH 75     // 高血量阈值
#endif
#ifndef ROBOT_BLOOD_LOW
#define ROBOT_BLOOD_LOW 40      // 低血量阈值
#endif
#ifndef ROBOT_BLOOD_DANGER
#define ROBOT_BLOOD_DANGER 20   // 危险血量阈值
#endif
#ifndef ROBOT_BLOOD_FULL
#define ROBOT_BLOOD_FULL 100    // 满血阈值
#endif

// 机器人状态等级评估函数
static uint8_t RobotstatelevelDecision(int quantity, int state)
{
    if (quantity == 3 && state > ROBOT_BLOOD_PERFECT) { return 1; }
    else if (quantity == 3 && state > ROBOT_BLOOD_HIGH) { return 2; }
    else if (quantity == 3) { return 3; }
    else if (quantity == 2 && state > ROBOT_BLOOD_HIGH) { return 4; }
    else if (quantity == 2) { return 5; }
    else if (quantity == 1) { return 6; }
    else { return 0; }
}

// 兵种打击决策函数
static int FireDecision(int is_success_enemypreempt)
{
    if (is_success_enemypreempt) {
        return 1;  // 防御fire模式
    } else {
        return 2;  // 进攻fire模式
    }
}

// 获取导航命令描述
static std::string GetNavDescription(int cmd) {
    switch(cmd) {
        case 0: return "回出生点";
        case 1: return "1点巡航";
        case 2: return "2点巡航/辅助游击";
        case 3: return "尝试占点";
        case 4: return "激进fire模式";
        default: return "未知命令";
    }
}
static int GyroscopeDecision(double x, double y)
{
    if (x>4.5 && y<-4.5){  
        return 4;  // 需要调整姿态
    } else if(x>4.5 && y>-4.5 && y<4.5){
        return 3; } // 姿态正常
    
    else if(2<x && x<4.5 && 2<y && y<4.5){return 2;} // gyroscope异常
    else if(1<x && x<2 && 1<y && y<2){return 1;}
    else{
        return 0; // 无效输入
    }
}
// 决策情况1分析（适用于状态较差的情况：leve(l 5-6）
static int PreemptSituation_1(int enemypreempt, int ourpreempt, int blood)
{
    if (enemypreempt) {
        if (blood < ROBOT_BLOOD_DANGER) { return 0; }
        if (blood > ROBOT_BLOOD_HIGH) { return 3; }
        else if (blood > ROBOT_BLOOD_LOW) { return 1; }
    }
    else {
        if (blood < ROBOT_BLOOD_DANGER) { return 0; }
        if (blood > ROBOT_BLOOD_HIGH) {
            if (ourpreempt) { return 2; }
            else { return 3; }
        }
        else if (blood > ROBOT_BLOOD_LOW) { return 2; }
    }
    return 0;
}

// 决策情况2分析（适用于中等状态：level 3-4）
static int PreemptSituation_2(int enemypreempt, int ourpreempt, int blood)
{
    if (enemypreempt) {
        if (blood < ROBOT_BLOOD_LOW) { return 0; }
        else if (blood > ROBOT_BLOOD_FULL) { return 3; }
    }
    else {
        if (blood < ROBOT_BLOOD_LOW) { return 0; }
        else if (blood > ROBOT_BLOOD_FULL) {
            if (ourpreempt) { return 2; }
            else { return 3; }
        }
    }
    return 0;
}

// 决策情况3分析（适用于优势状态：level 1-2）
static int PreemptSituation_3(int enemypreempt, int ourpreempt, int blood)
{
    if (enemypreempt) {
        if (blood < ROBOT_BLOOD_LOW) { return 0; }
        else if (blood > ROBOT_BLOOD_FULL) { return 3; }
    }
    else {
        if (blood < ROBOT_BLOOD_LOW) { return 0; }
        else if (blood > ROBOT_BLOOD_FULL) {
            if (ourpreempt) { return 3; }
            else { return 3; }
        }
    }
    return 0;
}

// 主决策树函数
CmdTreeOutput CmdDecisionTree(const CmdTreeInput &input)
{
    CmdTreeOutput output;
    output.vision_nav_cmd = 0;
    output.fire_decision = 0;
    output.description = "";
 
    // 根据 odometry 位置计算陀螺仪/姿态决策
    output.gyroscope = static_cast<uint8_t>(GyroscopeDecision(input.odom_x, input.odom_y));

    // 计算机器人状态等级（1-6，基于数量和血量）
    uint8_t is_robot_state_level = RobotstatelevelDecision(
        input.current_robot_quantity, 
        input.blood_warn_state
    );
    
    // 只在比赛进行中执行决策
    if (input.game_process == 4) {
        // 根据整体战力调整状态等级
        is_robot_state_level += input.is_ALL_state_level;
        if (is_robot_state_level > 6) { is_robot_state_level = 6; }
        else if (is_robot_state_level < 1) { is_robot_state_level = 1; }

        output.robot_state_level = is_robot_state_level;

        // 计算火力决策
        output.fire_decision = FireDecision(input.is_success_enemypreempt);

        // 判断阶段
        std::string phase = (input.current_state_left_time <= 60.f) ? "最后60秒" : "前期阶段";
        
        // 根据状态等级执行相应决策
        switch (is_robot_state_level) {
            case 1:
            case 2:  // 优势状态->激进策略
                output.vision_nav_cmd = PreemptSituation_3(input.is_success_enemypreempt, 
                                                          input.is_success_ourpreempt, 
                                                          input.blood_warn_state);
                output.description = phase + "-优势状态(激进):" + GetNavDescription(output.vision_nav_cmd);
                break;
            case 3:
            case 4:  // 中等状态->平衡策略
                output.vision_nav_cmd = PreemptSituation_2(input.is_success_enemypreempt, 
                                                          input.is_success_ourpreempt, 
                                                          input.blood_warn_state);
                output.description = phase + "-中等状态(平衡):" + GetNavDescription(output.vision_nav_cmd);
                break;
            case 5:
            case 6:  // 劣势状态->保守策略
                output.vision_nav_cmd = PreemptSituation_1(input.is_success_enemypreempt, 
                                                          input.is_success_ourpreempt, 
                                                          input.blood_warn_state);
                output.description = phase + "-劣势状态(保守):" + GetNavDescription(output.vision_nav_cmd);
                break;
            default:
                output.description = "未知状态";
                break;
        }
    } else {
        output.description = "比赛未进行";
    }

    return output;
}
