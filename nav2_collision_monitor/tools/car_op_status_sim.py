#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""模拟行为解析模块：按 V0.2 协议周期发布 /robot_move_state
（byd_custom_msgs/msg/RobotMoveState）。

只发布状态，不做任何碰撞判定、不生成速度命令。可选用 --mock-motion 同时发布
/motion_state（byd_custom_msgs/msg/MotionState），让状态消费端脱离整车也能跑通。

ooii: 2026-11 接口从 std_msgs/Int32（CarGmstatus 0–7）换成
byd_custom_msgs/RobotMoveState（uint8 robot_move_state 九值 0–8，V0.2 定稿，
QoS RELIABLE），话题 /robot_move_state，发布周期 100 ms。change_seq 在每次
状态切换时 +1，周期重发保持不变。

九值 → 9 个设计状态的映射在 car_op_zones.py 里；本脚本只负责按接口原样发状态。
"""
import argparse
import math
import sys
import time


# ooii: RobotMoveState V0.2 的九值（与 msg 里常量一一对应）。
class RobotMoveStateValue:
    UNKNOWN = 0
    MOVING_FORWARD = 1
    MOVING_BACKWARD = 2
    MOVING_FORWARD_TO_PICK = 3
    MOVING_FORWARD_TO_PLACE = 4
    MOVING_DIR_UNKNOWN = 5
    ROTATING = 6
    STATIONARY = 7
    PAUSED = 8

ALL_STATES = tuple(range(0, 9))

# ooii: 状态值 → 中文名，仅用于日志。
NAME_OF = {
    RobotMoveStateValue.UNKNOWN: '未知',
    RobotMoveStateValue.MOVING_FORWARD: '正走',
    RobotMoveStateValue.MOVING_BACKWARD: '倒走',
    RobotMoveStateValue.MOVING_FORWARD_TO_PICK: '正走去取',
    RobotMoveStateValue.MOVING_FORWARD_TO_PLACE: '正走去放',
    RobotMoveStateValue.MOVING_DIR_UNKNOWN: '行走但方向未知',
    RobotMoveStateValue.ROTATING: '原地旋转',
    RobotMoveStateValue.STATIONARY: '静止',
    RobotMoveStateValue.PAUSED: '任务暂停',
}

# 缺省周期：按接口文档的 100 ms。
DEFAULT_PERIOD = 0.1


def build_schedule(args):
    """返回 [(枚举值, 该段保持秒数), ...]，构成一次完整循环。"""
    if args.mode == 'hold':
        return [(args.value, float('inf'))]
    if args.mode == 'script':
        return load_script(args.file)
    order = list(ALL_STATES)   # ooii: 九值 0–8 全排进周期
    return [(v, args.hold) for v in order]


def load_script(stream):
    """CSV：每行 `枚举值,保持秒数`，# 开头与空行忽略。

    stream 是 --file 用 FileType 打开的文件对象，报错时取其 name 用于定位。
    """
    path = getattr(stream, 'name', '<script>')
    schedule = []
    with stream:
        for lineno, raw in enumerate(stream, 1):
            line = raw.split('#', 1)[0].strip()
            if not line:
                continue
            parts = [p.strip() for p in line.split(',')]
            if len(parts) != 2:
                raise ValueError(f'{path}:{lineno}: 需要 "枚举值,保持秒数"')
            value, hold = int(parts[0]), float(parts[1])
            if not math.isfinite(hold) or hold <= 0:
                raise ValueError(f'{path}:{lineno}: 保持秒数须为有限正数')
            schedule.append((value, hold))
    if not schedule:
        raise ValueError(f'{path}: 没有任何有效行')
    return schedule


def mock_motion_for(value, args, elapsed):
    """给出该状态值下模拟的 (v_car, w_car, lift_height)。

    ooii: V0.2 九值自带方向（正走/倒走），这里按状态造对应的 v 符号；
    --direction 只控制原地旋转的转向。进站（去取/去放）给慢速，
    未知/暂停/静止不造速度，不代表整车动力学。
    """
    if value == RobotMoveStateValue.ROTATING:
        return 0.0, args.mock_w * (1 if args.direction >= 0 else -1), 0.0
    if value in (RobotMoveStateValue.UNKNOWN,
                 RobotMoveStateValue.MOVING_DIR_UNKNOWN,
                 RobotMoveStateValue.STATIONARY,
                 RobotMoveStateValue.PAUSED):
        return 0.0, 0.0, 0.0
    if value in (RobotMoveStateValue.MOVING_FORWARD_TO_PICK,
                 RobotMoveStateValue.MOVING_FORWARD_TO_PLACE):
        return args.mock_v * 0.3, 0.0, 0.0       # 进站：慢速前进
    if value == RobotMoveStateValue.MOVING_BACKWARD:
        return -args.mock_v, 0.0, 0.0
    return args.mock_v, 0.0, 0.0                 # 正走


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--mode', choices=['cycle', 'hold', 'script'], default='cycle')
    parser.add_argument('--value', type=int,
                        default=RobotMoveStateValue.STATIONARY,
                        help='ooii: --mode hold 时固定发布的 robot_move_state 0–8'
                             '（默认 7 静止）')
    parser.add_argument('--file', type=argparse.FileType('r', encoding='utf-8'),
                        help='--mode script 时的 CSV 路径')
    parser.add_argument('--hold', type=float, default=3.0,
                        help='--mode cycle 时每个枚举值的保持秒数')
    parser.add_argument('--period', type=float, default=DEFAULT_PERIOD,
                        help='发布周期，秒；接口文档为 0.1')
    parser.add_argument('--topic', default='/robot_move_state')
    parser.add_argument('--mock-motion', action='store_true',
                        help='同时模拟发布 /motion_state，让消费端脱离整车可跑')
    parser.add_argument('--motion-topic', default='/motion_state')
    parser.add_argument('--mock-v', type=float, default=0.3,
                        help='--mock-motion 下带 v 的状态发布的 |v_car|')
    parser.add_argument('--mock-w', type=float, default=1.0,
                        help='--mock-motion 下原地旋转发布的 |w_car|')
    parser.add_argument('--direction', type=int, choices=[1, -1], default=1,
                        help='ooii: 原地旋转的转向（1 逆时针 / -1 顺时针）；'
                             '行走方向由九值状态本身决定')
    parser.add_argument('--no-repeat', action='store_true',
                        help='周期跑完一轮后退出，便于脚本化验证')
    parser.add_argument('--yield-to-other-publishers', action='store_true',
                        help='检测到另一个同话题发布者时暂停自己，外部源退出后恢复；'
                             '供 launch 的默认状态源让位给手动状态窗口')
    # 由 launch 启动时会追加 --ros-args … --params-file，交给 ROS 处理，
    # 不当作本脚本的参数（否则 argparse 直接退出）。
    args, _ros_argv = parser.parse_known_args(argv)

    if not math.isfinite(args.period) or args.period <= 0:
        parser.error('--period 须为有限正数')
    if args.mode == 'script' and args.file is None:
        parser.error('--mode script 需要 --file')
    try:
        schedule = build_schedule(args)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))

    # rclpy 只在这里导入：--help 与参数校验不依赖 ROS 环境。
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    # ooii: 状态接口改为 RobotMoveState（Header + uint8 robot_move_state）。
    from byd_custom_msgs.msg import RobotMoveState

    rclpy.init(args=None)
    node = Node('car_op_status_sim')

    # ooii: V0.2 协议规定 QoS RELIABLE；与 car_op_zones 的订阅端一致。
    status_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                            reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.VOLATILE)
    status_pub = node.create_publisher(RobotMoveState, args.topic, status_qos)

    motion_pub = None
    motion_cls = None
    if args.mock_motion:
        from byd_custom_msgs.msg import MotionState
        motion_cls = MotionState
        # 与 myworld_bringup/odom_to_motion_state.py 的 profile 保持一致。
        motion_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                                reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.VOLATILE)
        motion_pub = node.create_publisher(MotionState, args.motion_topic, motion_qos)

    state = {'index': 0, 'segment_start': time.monotonic(), 'cycles': 0,
             'yielded': False, 'last_value': None, 'seq': 0}
    total = len(schedule)

    def publish():
        now = time.monotonic()
        # 默认状态源让位：自己也是一个发布者，所以 >1 表示有外部源。
        # 外部源退出后重新接回，并重置当前段计时，避免补发已过期的切换。
        if args.yield_to_other_publishers:
            if node.count_publishers(args.topic) > 1:
                if not state['yielded']:
                    state['yielded'] = True
                    node.get_logger().info(
                        f'检测到另一个 {args.topic} 发布者，暂停默认状态源')
                return
            if state['yielded']:
                state['yielded'] = False
                state['segment_start'] = now
                node.get_logger().info(
                    f'外部 {args.topic} 发布者已退出，恢复默认状态源')
        value, hold = schedule[state['index']]
        if now - state['segment_start'] >= hold:
            state['index'] += 1
            state['segment_start'] = now
            if state['index'] >= total:
                state['cycles'] += 1
                if args.no_repeat:
                    node.get_logger().info(
                        f'周期跑完 {total} 段，--no-repeat 生效，退出')
                    raise KeyboardInterrupt
                state['index'] = 0
            value, hold = schedule[state['index']]

        if value not in NAME_OF:
            node.get_logger().warn(
                f'robot_move_state={value} 不在 RobotMoveState 定义内（0–8），'
                f'仍照发（消费端会告警）')

        # ooii: change_seq 只在状态切换时 +1，周期重发保持不变。
        if state['last_value'] is not None and value != state['last_value']:
            state['seq'] = (state['seq'] + 1) & 0xFFFFFFFF
        state['last_value'] = value

        status_msg = RobotMoveState()
        status_msg.header.stamp = node.get_clock().now().to_msg()
        status_msg.header.frame_id = 'base_link'   # ooii: 状态无坐标含义，仅填车身系
        status_msg.robot_move_state = int(value)
        status_msg.change_seq = int(state['seq'])
        status_pub.publish(status_msg)

        detail = ''
        if motion_pub is not None:
            v, w, lift = mock_motion_for(value, args, now - state['segment_start'])
            msg = motion_cls()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.v_car = float(v)
            msg.w_car = float(w)
            msg.v_lift = 0.0
            msg.lift_height = float(lift)
            msg.w_shelf = 0.0
            msg.yaw_shelf = 0.0
            motion_pub.publish(msg)
            detail = f'  motion: v={v:+.2f} w={w:+.2f} lift={lift:.2f}'

        node.get_logger().info(
            f'{args.topic} <- {value} ({NAME_OF.get(value, "未定义")}) '
            f'seq={state["seq"]}'
            f'  第{state["cycles"] + 1}轮 段{state["index"] + 1}/{total}{detail}',
            throttle_duration_sec=0.0)

    timer = node.create_timer(args.period, publish)
    node.get_logger().info(
        f'开始发布：mode={args.mode} 周期={args.period * 1000:.0f}ms '
        f'共{total}段 mock_motion={args.mock_motion}')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        # SIGTERM／Ctrl+C 正常收尾；不打印 traceback。
        pass
    finally:
        node.destroy_timer(timer)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
