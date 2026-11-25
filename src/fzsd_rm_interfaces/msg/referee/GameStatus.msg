# msg for referee system (V1.6.4 0x0001)

# constants for game progress
uint8 NOT_START = 0      # 游戏未开始
uint8 PREPARATION = 1    # 准备阶段（机器人自由控制，禁用裁判系统）
uint8 SELF_CHECKING = 2  # 裁判系统自检阶段（无法控制机器人，并初始化所有裁判系统数据）
uint8 COUNT_DOWN = 3     # 5秒倒计时阶段（无法控制机器人，启用裁判系统）
uint8 RUNNING = 4        # 运行阶段（机器人自由控制，启用裁判系统）
uint8 GAME_OVER = 5      # 游戏结束阶段（无法控制机器人，禁用裁判系统）


uint8 game_progress 0
int32 stage_remain_time 0           # seconds