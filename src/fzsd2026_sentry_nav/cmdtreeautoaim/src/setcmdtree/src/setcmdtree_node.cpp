#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "setcmdtree/cmdtree.hpp"
#include <sstream>
#include <iostream>

class SetCmdTreeNode : public rclcpp::Node
{
public:
  SetCmdTreeNode()
  : Node("setcmdtree_node")
  {
    // 订阅器：监听 'setcmdtree' 话题接收外部命令
    subscriber_ = this->create_subscription<std_msgs::msg::String>(
      "setcmdtree", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        processCommand(msg->data);
      }
    );

    // 订阅器：订阅 odometry 话题获取机器人当前位置
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odometry", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_odom_x_ = msg->pose.pose.position.x;
        current_odom_y_ = msg->pose.pose.position.y;
      }
    );
    
    // 发布器：发布火力决策结果到 'cmdtreeautoaim' 话题
    publisher_ = this->create_publisher<std_msgs::msg::String>("cmdtreeautoaim", 10);
    // 发布器：发布导航命令到 '/cmdtree/navigation' 话题
    nav_command_publisher_ = this->create_publisher<std_msgs::msg::UInt8>("/cmdtree/navigation", 10);
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "......\\  /.......\\ /.......\\ /.......\\ /.......");
    RCLCPP_INFO(this->get_logger(), "......_  _......._  _......._  _......._  _......");
    RCLCPP_INFO(this->get_logger(), ".....(.o o.)....(.o o.)....(.o o.)....(.o o.)....");
    RCLCPP_INFO(this->get_logger(), "..../..V.V..\\../..V.V..\\../..V.V..\\../..V.V..\\...");
    RCLCPP_INFO(this->get_logger(), ".....\\_/ \\_/....\\_/ \\_/....\\_/ \\_/....\\_/ \\_/......");
    RCLCPP_INFO(this->get_logger(), "Cockroach invasion!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    RCLCPP_INFO(this->get_logger(), "SetCmdTree节点已启动");
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "输入格式示例:");
    RCLCPP_INFO(this->get_logger(), "quantity:3,blood:85,game:4,time:120,our:1,enemy:0,level:0");
    RCLCPP_INFO(this->get_logger(), "参数说明:");
    RCLCPP_INFO(this->get_logger(), "  quantity: 当前机器人数量 (1-3)");
    RCLCPP_INFO(this->get_logger(), "  blood:    血量状态 (0-100)");
    RCLCPP_INFO(this->get_logger(), "  game:     比赛状态 (4=进行中)");
    RCLCPP_INFO(this->get_logger(), "  time:     剩余时间(秒)");
    RCLCPP_INFO(this->get_logger(), "  our:      我方占点 (0/1)");
    RCLCPP_INFO(this->get_logger(), "  enemy:    敌方占点 (0/1)");
    RCLCPP_INFO(this->get_logger(), "  level:    战力评估 (-1/0/1)");
    RCLCPP_INFO(this->get_logger(), "========================================");
  }

private:
  void processCommand(const std::string& cmd)
  {
    RCLCPP_INFO(this->get_logger(), "收到输入: %s", cmd.c_str());
    
    // 解析输入
    CmdTreeInput input;
    if (!parseInput(cmd, input)) {
      RCLCPP_ERROR(this->get_logger(), "ERROR");
      return;
    }

    // 将最新 odometry 位置填入 input
    input.odom_x = current_odom_x_;
    input.odom_y = current_odom_y_;
    
    // 显示解析后的输入
    RCLCPP_INFO(this->get_logger(), "输入参数:");
    RCLCPP_INFO(this->get_logger(), "  ├─ 机器人数量: %d", input.current_robot_quantity);
    RCLCPP_INFO(this->get_logger(), "  ├─ 血量状态: %d", input.blood_warn_state);
    RCLCPP_INFO(this->get_logger(), "  ├─ 比赛状态: %d", input.game_process);
    RCLCPP_INFO(this->get_logger(), "  ├─ 剩余时间: %.1f秒", input.current_state_left_time);
    RCLCPP_INFO(this->get_logger(), "  ├─ 我方占点: %d", input.is_success_ourpreempt);
    RCLCPP_INFO(this->get_logger(), "  ├─ 敌方占点: %d", input.is_success_enemypreempt);
    RCLCPP_INFO(this->get_logger(), "  ├─ 战力评估: %d", input.is_ALL_state_level);
    RCLCPP_INFO(this->get_logger(), "  ├─ 里程计 X: %.3f", input.odom_x);
    RCLCPP_INFO(this->get_logger(), "  └─ 里程计 Y: %.3f", input.odom_y);
    
    // 执行决策树
    CmdTreeOutput output = CmdDecisionTree(input);
    
    // 发布火力决策结果到 'cmdtreeautoaim' 话题
    auto fire_msg = std_msgs::msg::String();
    fire_msg.data = std::to_string(output.fire_decision);
    publisher_->publish(fire_msg);
    RCLCPP_INFO(this->get_logger(), "Cockroach get /cmdtreeautoaim: %s", fire_msg.data.c_str());
    
    // 发布导航命令到 '/cmdtree/navigation' 话题
    auto nav_msg = std_msgs::msg::UInt8();
    nav_msg.data = static_cast<uint8_t>(output.vision_nav_cmd);
    nav_command_publisher_->publish(nav_msg);
    RCLCPP_INFO(this->get_logger(), "Cockroach get /cmdtree/navigation: %u", nav_msg.data);
    
    // 显示输出结果
    RCLCPP_INFO(this->get_logger(), "决策输出:");
    RCLCPP_INFO(this->get_logger(), "  ├─ 状态等级: %d", output.robot_state_level);
    RCLCPP_INFO(this->get_logger(), "  ├─ 导航命令: %d", output.vision_nav_cmd);
    RCLCPP_INFO(this->get_logger(), "  ├─ 火力决策: %d", output.fire_decision);
    RCLCPP_INFO(this->get_logger(), "  ├─ 陀螺仪决策: %d", output.gyroscope);
    RCLCPP_INFO(this->get_logger(), "  └─ 决策描述: %s", output.description.c_str());
  }

  bool parseInput(const std::string& cmd, CmdTreeInput& input)
  {
    // 默认值
    input.current_robot_quantity = 3;
    input.blood_warn_state = 100;
    input.game_process = 4;
    input.current_state_left_time = 120.0f;
    input.is_success_ourpreempt = 0;
    input.is_success_enemypreempt = 0;
    input.is_ALL_state_level = 0;
    
    std::istringstream iss(cmd);
    std::string token;
    
    while (std::getline(iss, token, ',')) {
      size_t pos = token.find(':');
      if (pos == std::string::npos) continue;
      
      std::string key = token.substr(0, pos);
      std::string value = token.substr(pos + 1);
      
      try {
        if (key == "quantity") {
          input.current_robot_quantity = std::stoi(value);
        } else if (key == "blood") {
          input.blood_warn_state = std::stoi(value);
        } else if (key == "game") {
          input.game_process = std::stoi(value);
        } else if (key == "time") {
          input.current_state_left_time = std::stof(value);
        } else if (key == "our") {
          input.is_success_ourpreempt = std::stoi(value);
        } else if (key == "enemy") {
          input.is_success_enemypreempt = std::stoi(value);
        } else if (key == "level") {
          input.is_ALL_state_level = std::stoi(value);
        }
      } catch (...) {
        return false;
      }
    }
    
    return true;
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr nav_command_publisher_;

  double current_odom_x_ = 0.0;
  double current_odom_y_ = 0.0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SetCmdTreeNode>());
  rclcpp::shutdown();
  return 0;
}
