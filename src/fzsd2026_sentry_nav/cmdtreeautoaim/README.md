<!-- Here is  zhang zhang's README document
......\  /.......\  /.......\  /.......\  /.......\  /........
......_  _......._  _......._  _......._  _......._  _........
.....(.o o.)....(.o o.)....(.o o.)....(.o o.)....(.o o.)......
..../..V.V..\../..V.V..\../..V.V..\../..V.V..\../..V.V..\.....
.....\_/ \_/....\_/ \_/....\_/ \_/....\_/ \_/....\_/ \_/......
Cockroach invasion!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! -->


<!-- 决策树描述捏 -->
<!-- 
每次运行，会问打击目标点，每次输入一个数字话题就循环发布那个数字捏

话题名为cmdtreeautoaim

局部静态变量（在 CmdDecisionTree 函数内部）
is_returing_blood - static uint8_t

返回补血状态标志（未使用，保留变量）
is_success_ourpreempt - static uint8_t

我方占点成功标志（0=未占点，非0=已占点）
is_success_enemypreempt - static uint8_t

敌方占点成功标志（0=敌方未占点，非0=敌方已占点）
is_robot_state_level - static uint8_t

机器人自身生存环境评级（1-6级，数字越大状态越差）
1 = perfect（完美：3机器人+血量>90）
2 = great（优秀：3机器人+血量>75）
3 = good（良好：3机器人）
4 = fine（尚可：2机器人+血量>75）
5 = just（勉强：2机器人）
6 = poor（糟糕：1机器人）
0 = free state（自由状态）
is_ALL_state_level - static uint8_t

敌我双方整体战力评估的调整系数（手动设置）
-1 = 我方优于敌方
0 = 我方约等于敌方
1 = 我方劣于敌方
stay_start_time - static time_t

停留机制开始计时的时间戳
stay_state - static uint8_t

停留状态标志
1 = 正在停留（未超时）
0 = 停留超时，应离开
函数参数变量
RobotstatelevelDesicion 函数参数：
quantity - int

当前存活的机器人数量
state - int

机器人血量警告状态值
PreemptSituation_1/2/3 函数参数：
enemypreempt - int

敌方是否占点（布尔型，0=否，非0=是）
ourpreempt - int

我方是否占点（布尔型，0=否，非0=是）
blood - int

当前血量状态值
UpdateStayState 函数局部变量：
now - time_t
当前时间戳（用于计算停留时长）
FireDecision 函数参数：
is_success_enemypreempt - int
敌方占点状态（用于决定防御或进攻打击模式）
外部引用变量（来自 chassis_fetch_data 结构体）
chassis_fetch_data.current_robot_quantity - 类型未知

当前存活机器人数量（从底盘数据获取）
chassis_fetch_data.blood_warn_state - 类型未知

血量警告状态（从底盘数据获取）
chassis_fetch_data.game_process - 类型未知

比赛进程状态（4=比赛进行中）
chassis_fetch_data.current_state_left_time - float

当前阶段剩余时间（秒）
≤60 = 最后60秒阶段
60 = 前期阶段

使用的常量（宏定义，未在此文件中定义）
ROBOT_BLOOD_PERFECT - 血量完美阈值（通常为90）
ROBOT_BLOOD_HIGH - 血量高阈值（通常为75）
ROBOT_BLOOD_LOW - 血量低阈值（通常为40）
ROBOT_BLOOD_DANGER - 血量危险阈值（通常为20）
ROBOT_BLOOD_FULL - 血量满值阈值（通常为100） -->

<!-- 要打开4个终端，按顺序执行 -->

bash'''
<!-- 终端1: 编译并启动发布节点 -->
colcon build
source install/setup.bash
ros2 run cmdtreeautoaim number_publisher

<!-- 终端2: 启动决策树节点 (等待终端1启动完成后再执行) -->
source install/setup.bash
ros2 run setcmdtree setcmdtree_node

<!-- 终端3: 查看火力决策输出 (等待终端1启动完成后再执行) -->
source install/setup.bash
ros2 topic echo /fire_command

<!-- 终端4: 发布测试数据 -->
source install/setup.bash
ros2 topic pub /setcmdtree std_msgs/msg/String "data: 'quantity:2,blood:45,game:4,time:55,our:0,enemy:1,level:1'" -1
'''
<!-- 命令依次输入
机器人数量: 1-3
血量状态: 0-100
比赛状态: 1/0
剩余时间: 0-180
我方占点: 1/0
敌方占点: 1/0
战力评估: -1/0/1
。 -->