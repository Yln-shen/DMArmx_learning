#!/usr/bin/env python3
"""用 MoveIt 的 /compute_ik 把末端挪一个小位移（保持姿态），再用 JTC 执行。

比直接发关节角安全：位移是你指定的，撞不撞由 IK/规划决定的部分我们来控制。

用法:
    # 沿 base_link 的 +Z 抬高 0.12 m
    pixi run python src/test/move_ik.py --dz 0.12
    # 沿 +X 后退 0.1 m
    pixi run python src/test/move_ik.py --dx -0.1
"""
import argparse
import sys
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from rclpy.duration import Duration
from rclpy.time import Time

from arm_move import ARM_JOINTS, ArmMover


class IkMover(ArmMover):
    def __init__(self):
        super().__init__()
        self.ik = self.create_client(GetPositionIK, '/compute_ik')
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=2))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def current_pose(self, base='base_link', tip='link6', timeout=5.0):
        t = None
        t0 = time.time()
        while time.time() - t0 < timeout:
            try:
                t = self.tf_buffer.lookup_transform(base, tip, Time())
                break
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.1)
        if t is None:
            raise RuntimeError(f'拿不到 {base} -> {tip} 的 TF')
        p = PoseStamped()
        p.header.frame_id = base
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = t.transform.translation.x
        p.pose.position.y = t.transform.translation.y
        p.pose.position.z = t.transform.translation.z
        p.pose.orientation = t.transform.rotation
        return p

    def solve_ik(self, pose):
        if not self.ik.wait_for_service(timeout_sec=10.0):
            print('没有 /compute_ik（MoveIt 没起？）')
            return None
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'arm'
        req.ik_request.ik_link_name = 'link6'
        req.ik_request.pose_stamped = pose
        req.ik_request.timeout.sec = 1
        req.ik_request.avoid_collisions = False
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            print('IK 失败, code =', None if res is None else res.error_code.val)
            return None
        return dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dx', type=float, default=0.0)
    ap.add_argument('--dy', type=float, default=0.0)
    ap.add_argument('--dz', type=float, default=0.0)
    args = ap.parse_args()

    rclpy.init()
    node = IkMover()
    st = node.wait_state()
    if st is None:
        sys.exit('没有 /joint_states')
    time.sleep(0.5)
    rclpy.spin_once(node, timeout_sec=0.2)

    pose = node.current_pose()
    print(f'当前末端: ({pose.pose.position.x:+.3f}, {pose.pose.position.y:+.3f}, {pose.pose.position.z:+.3f})')
    pose.pose.position.x += args.dx
    pose.pose.position.y += args.dy
    pose.pose.position.z += args.dz

    sol = node.solve_ik(pose)
    if sol is None:
        return
    targets = {j: float(sol[j]) for j in ARM_JOINTS if j in sol}
    print('IK 解:', {k: round(v, 3) for k, v in targets.items()})
    ok = node.move_jtc(targets)
    print('移动成功' if ok else '移动失败')


if __name__ == '__main__':
    main()
