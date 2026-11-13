#!/usr/bin/env python3
"""用 MoveIt (/move_action) 做关节空间移动的小工具。

用法:
  pixi run python src/test/arm_move.py --show                       # 打印当前关节角
  pixi run python src/test/arm_move.py --set joint6=0.5,joint5=0.1  # 移动到指定关节角
"""
import argparse
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, JointConstraint, MoveItErrorCodes,
                             PlanningOptions)

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']


class ArmMover(Node):
    def __init__(self):
        super().__init__('arm_mover')
        self.current = None
        self.create_subscription(JointState, '/joint_states', self.on_js, 10)
        self.client = ActionClient(self, MoveGroup, '/move_action')
        self.jtc = ActionClient(self, FollowJointTrajectory,
                                '/arm_controller/follow_joint_trajectory')

    def on_js(self, msg):
        self.current = dict(zip(msg.name, msg.position))

    def wait_state(self, timeout=10.0):
        t0 = time.time()
        while self.current is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.current

    def move_jtc(self, targets, timeout=30.0):
        """直接给 JointTrajectoryController 发一条关节轨迹（不经 MoveIt 规划）。

        只适合在当前姿态附近做小幅移动：没有碰撞检查，但稳定、不会因为
        move_group 的状态校验失败。
        """
        base = self.wait_state()
        if base is None:
            print('拿不到 /joint_states')
            return False
        if not self.jtc.wait_for_server(timeout_sec=10.0):
            print('找不到 /arm_controller/follow_joint_trajectory')
            return False

        deltas = {j: float(targets.get(j, base[j])) - base[j] for j in ARM_JOINTS}
        max_delta = max(abs(d) for d in deltas.values())
        duration = max(2.0, max_delta * 2.5)      # 慢一点，约 0.4 rad/s

        traj = JointTrajectory()
        traj.joint_names = ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(targets.get(j, base[j])) for j in ARM_JOINTS]
        pt.velocities = [0.0] * len(ARM_JOINTS)
        pt.time_from_start.sec = int(duration)
        pt.time_from_start.nanosec = int((duration % 1) * 1e9)
        traj.points.append(pt)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        fut = self.jtc.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        gh = fut.result()
        if gh is None or not gh.accepted:
            print('轨迹被拒绝')
            return False
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res, timeout_sec=timeout)
        if res.result() is None:
            print('执行超时')
            return False
        code = res.result().result.error_code
        time.sleep(0.3)
        self.current = None
        self.wait_state()
        if code != 0:
            print(f'控制器返回 error_code={code}')
            return False
        return True

    def move(self, targets, vel=0.15, acc=0.15, timeout=30.0):
        """targets: {joint_name: position}，返回是否成功。"""
        if not self.client.wait_for_server(timeout_sec=10.0):
            print('找不到 /move_action（MoveIt 没起？）')
            return False
        base = self.wait_state()
        if base is None:
            print('拿不到 /joint_states')
            return False

        goal = MoveGroup.Goal()
        goal.request.group_name = 'arm'
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = vel
        goal.request.max_acceleration_scaling_factor = acc

        cons = Constraints()
        for name in ARM_JOINTS:
            target = float(targets.get(name, base[name]))
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = target
            jc.tolerance_above = 0.02
            jc.tolerance_below = 0.02
            jc.weight = 1.0
            cons.joint_constraints.append(jc)
        goal.request.goal_constraints.append(cons)

        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2

        fut = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        gh = fut.result()
        if gh is None or not gh.accepted:
            print('目标被拒绝')
            return False
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res, timeout_sec=timeout)
        if res.result() is None:
            print('执行超时')
            return False
        code = res.result().result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            print(f'MoveIt 失败, error_code={code}')
            return False
        return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--show', action='store_true')
    ap.add_argument('--set', default='')
    ap.add_argument('--vel', type=float, default=0.15)
    args = ap.parse_args()

    rclpy.init()
    node = ArmMover()
    state = node.wait_state()
    if state is None:
        print('没有 /joint_states')
        sys.exit(1)

    if args.show or not args.set:
        for j in ARM_JOINTS:
            print(f'{j}: {state[j]:+.4f}')
        return

    targets = {}
    for item in args.set.split(','):
        k, v = item.split('=')
        targets[k.strip()] = float(v)
    ok = node.move(targets, vel=args.vel)
    print('移动成功' if ok else '移动失败')
    if ok:
        time.sleep(0.5)
        node.wait_state()
        for j in ARM_JOINTS:
            print(f'{j}: {node.current[j]:+.4f}')


if __name__ == '__main__':
    main()
