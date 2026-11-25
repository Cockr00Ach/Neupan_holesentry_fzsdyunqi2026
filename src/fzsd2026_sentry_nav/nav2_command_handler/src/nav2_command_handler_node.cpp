#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class NavCommandMapper : public rclcpp::Node
{
public:
  NavCommandMapper() : Node("nav_command_mapper")
  {
    // [测试阶段] 订阅 /cmdtree/navigation 话题，接收来自决策树(cmdtree)的导航命令
    // 注意：正式比赛时此命令应来自下位机，当前仅用于测试决策树功能
    nav_command_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
      "/cmdtree/navigation", rclcpp::SensorDataQoS(),
      std::bind(&NavCommandMapper::nav_command_callback, this, std::placeholders::_1));

    goal_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/red_standard_robot1/goal_pose", 10);

    RCLCPP_INFO(this->get_logger(), "NavCommandMapper 节点已启动 [测试模式：仅接收cmdtree命令]");
    RCLCPP_INFO(this->get_logger(), "等待来自决策树(cmdtree)的导航命令 (/cmdtree/navigation)...");
  }

private:
  // [测试阶段] 回调函数：接收决策树(cmdtree)发布的导航命令
  // 注意：生产环境中此数据应由下位机通过串口通信发送
  void nav_command_callback(const std_msgs::msg::UInt8::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "[cmdtree] 收到导航命令: %u", msg->data);

    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.stamp = this->get_clock()->now();
    goal_pose.header.frame_id = "map";

    // 根据决策树命令映射到具体目标点
     if (msg->data == 0) {
      // 回出生点
      goal_pose.pose.position.x = 0;
      goal_pose.pose.position.y = 0;
      goal_pose.pose.position.z = 0;

    } else if (msg->data == 1) {
      // 1点巡航
      goal_pose.pose.position.x = 0.5;
      goal_pose.pose.position.y = 0.0;
      goal_pose.pose.position.z = 0;

    } else if (msg->data == 2) {
      // 2点巡航/辅助游击
      goal_pose.pose.position.x = 0.5;
      goal_pose.pose.position.y = 0.5;
      goal_pose.pose.position.z = 0;

    
    } else if (msg->data == 3) {
      // 尝试占点
      goal_pose.pose.position.x = 0;
      goal_pose.pose.position.y = 0.5;
      goal_pose.pose.position.z = 0;
    
    } else if (msg->data == 4) {
      // 激进fire模式
      goal_pose.pose.position.x = 2.8689160346984863;
      goal_pose.pose.position.y = -4.21600341796875;
      goal_pose.pose.position.z = -0.002277374267578125;
    
    } else if (msg->data == 5) {
      // 预留位置5
      goal_pose.pose.position.x = 4.744440078735352;
      goal_pose.pose.position.y = -0.5410155057907104;
      goal_pose.pose.position.z = -0.000370025634765625;

    } else {
      RCLCPP_WARN(this->get_logger(), "收到未知决策树命令: %d", msg->data);
      return;
    }

    // 统一设置方向参数
    goal_pose.pose.orientation.x = 0.0;
    goal_pose.pose.orientation.y = 0.0;
    goal_pose.pose.orientation.z = 0.0;
    goal_pose.pose.orientation.w = 1.0;

    RCLCPP_INFO(this->get_logger(), "[cmdtree测试] 命令[%u] -> 目标坐标：(%.3f, %.3f, %.3f)",
                msg->data,
                goal_pose.pose.position.x, 
                goal_pose.pose.position.y, 
                goal_pose.pose.position.z);

    goal_pose_publisher_->publish(goal_pose);
  }

  // [测试阶段] 订阅者：接收决策树(cmdtree)发布的导航命令
  // TODO: 正式比赛时改为接收下位机串口数据
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr nav_command_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_publisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavCommandMapper>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
