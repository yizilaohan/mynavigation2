# 机器人移动状态（Topic /robot_move_state，Manager -> 底盘避障节点，协议表12，qos RELIABLE）

# 值域重定义（设计稿《机器人移动状态发布设计（避障）V0.2》§2，2026-11-13 评审定稿）：

# 原 12 值负载模型（协议 2026-10-20 草案）裁撤——发布侧当时零消费者，值域重定义零风险窗口期；

# docx 表12 暂保留 12 值草案、由用户加注九值方案说明（拍板 N1），代码以本文件为准。

# 9~127 保留扩展段（负载维/充电站态回归预案），128~255 私有段；值一经落文只增不改。

std_msgs/Header header

uint8  robot_move_state                     # 机器人的移动状态
uint32 change_seq                           # 状态切换序号：每次切换 +1，周期重发不变
                                            # （消费者区分"新拍重发"与"新状态边沿"，V0.2 §5.1）

# 状态类型定义（九值）

uint8  STATE_UNKNOWN = 0                    # 未知/不可判（保守默认：启动未激活/运动输入失效/激活首拍前）
uint8  STATE_MOVING_FORWARD = 1             # 正走（车头朝行进方向）
uint8  STATE_MOVING_BACKWARD = 2            # 倒走（车尾朝行进方向；倒行去取/放亦报此值）
uint8  STATE_MOVING_FORWARD_TO_PICK = 3     # 正走去取（FORWARD ∧ 组终点站含抓取技能锚点）
uint8  STATE_MOVING_FORWARD_TO_PLACE = 4    # 正走去放（FORWARD ∧ 组终点站含放置技能锚点）
uint8  STATE_MOVING_DIR_UNKNOWN = 5         # 行走但方向未知（NO_LIMIT 未推断/加减速过渡带/无任务滑行）
uint8  STATE_ROTATING = 6                   # 原地旋转（v≈0 ∧ |w|>阈值）
uint8  STATE_STATIONARY = 7                 # 静止（空闲/到站技能/逗留等待/终态归零）
uint8  STATE_PAUSED = 8                     # 任务暂停（PAUSE 受理生效期，独立于 7 供避障调包络）
