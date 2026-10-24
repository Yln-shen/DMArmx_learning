#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include <tf2/LinearMath/Quaternion.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using MoveGroupInterface =
    moveit::planning_interface::MoveGroupInterface;

class Commander
{
public:
    Commander(const std::shared_ptr<rclcpp::Node>& node)
    {
        node_ = node;

        // 创建机械臂控制接口
        arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
        arm_->setMaxVelocityScalingFactor(1.0);
        arm_->setMaxAccelerationScalingFactor(1.0);

        // 创建夹爪控制接口
        gripper_ = std::make_shared<MoveGroupInterface>(node_, "gripper");
    }

    // 1. 前往 SRDF 中保存的命名姿态，例如 pose_1、home
    void goToNamedTarget(const std::string& name)
    {
        arm_->setStartStateToCurrentState();
        arm_->setNamedTarget(name);

        planAndExecute(arm_);
    }

    // 2. 前往指定关节角度
    void goToJointTarget(const std::vector<double>& joints)
    {
        arm_->setStartStateToCurrentState();
        arm_->setJointValueTarget(joints);

        planAndExecute(arm_);
    }

    // 3. 前往指定末端位姿
    // cartesian_path = false：普通规划，只保证到达终点
    // cartesian_path = true ：笛卡尔路径，末端尽量沿直线运动
    void goToPoseTarget(
        double x,
        double y,
        double z,
        double roll,
        double pitch,
        double yaw,
        bool cartesian_path = false)
    {
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q.normalize();

        geometry_msgs::msg::PoseStamped target_pose;

        // 使用机械臂当前的规划参考坐标系，通常就是 base_link
        const std::string reference_frame = arm_->getPlanningFrame();
        arm_->setPoseReferenceFrame(reference_frame);

        target_pose.header.frame_id = reference_frame;

        target_pose.pose.position.x = x;
        target_pose.pose.position.y = y;
        target_pose.pose.position.z = z;

        target_pose.pose.orientation.x = q.getX();
        target_pose.pose.orientation.y = q.getY();
        target_pose.pose.orientation.z = q.getZ();
        target_pose.pose.orientation.w = q.getW();

        arm_->setStartStateToCurrentState();

        // 普通位姿规划
        if (!cartesian_path)
        {
            arm_->setPoseTarget(target_pose);

            planAndExecute(arm_);

            // 清除本次位姿目标，避免影响后续规划
            arm_->clearPoseTargets();
        }
        // 笛卡尔路径规划
        else
        {
            std::vector<geometry_msgs::msg::Pose> waypoints;

            // 从当前末端位置，直线移动到 target_pose
            waypoints.push_back(target_pose.pose);

            moveit_msgs::msg::RobotTrajectory trajectory;
            moveit_msgs::msg::MoveItErrorCodes error_code;

            // 你的 MoveIt 版本需要 6 个参数
            double fraction = arm_->computeCartesianPath(
                waypoints,
                0.01,        // 每一步最大间隔：1 cm
                0.0,         // 关节跳变阈值
                trajectory,
                true,        // 是否避障
                &error_code  // 错误码输出
            );

            // fraction 接近 1，说明整条路径都规划成功
            if (fraction >= 0.999)
            {
                auto result = arm_->execute(trajectory);

                if (result != moveit::core::MoveItErrorCode::SUCCESS)
                {
                    RCLCPP_ERROR(
                        node_->get_logger(),
                        "Cartesian path execution failed!"
                    );
                }
            }
            else
            {
                RCLCPP_WARN(
                    node_->get_logger(),
                    "Cartesian path only completed %.1f%%",
                    fraction * 100.0
                );
            }
        }
    }

private:
    // 所有普通规划共用：规划成功才执行
    void planAndExecute(
        const std::shared_ptr<MoveGroupInterface>& interface)
    {
        MoveGroupInterface::Plan plan;

        bool success =
            (interface->plan(plan) ==
             moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            auto result = interface->execute(plan);

            if (result != moveit::core::MoveItErrorCode::SUCCESS)
            {
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "Trajectory execution failed!"
                );
            }
        }
        else
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Motion planning failed!"
            );
        }
    }

private:
    std::shared_ptr<rclcpp::Node> node_;

    std::shared_ptr<MoveGroupInterface> arm_;
    std::shared_ptr<MoveGroupInterface> gripper_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("commander");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::thread spinner([&executor]()
    {
        executor.spin();
    });

    {
        Commander commander(node);

        // 给 MoveIt 一点时间接收当前机械臂状态
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // 一次只保留一种测试方式，其他先注释掉

        // 方式 1：前往命名姿态
        commander.goToNamedTarget("pose_1");

        // 方式 2：前往指定关节角度
        // commander.goToJointTarget({1.5, 0.5, 0.0, 1.5, 0.0, -0.7});

        // 方式 3：普通末端位姿规划
        // commander.goToPoseTarget(
        //     0.0, -0.7, 0.4,
        //     3.14, 0.0, 0.0,
        //     false
        // );

        // 方式 4：笛卡尔路径，末端尽量直线到目标位置
        // commander.goToPoseTarget(
        //     0.0, -0.7, 0.4,
        //     3.14, 0.0, 0.0,
        //     true
        // );
    }

    executor.cancel();
    spinner.join();

    rclcpp::shutdown();

    return 0;
}
