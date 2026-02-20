#include <cmath>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace gazebo {
class TowingController : public ModelPlugin {
public:
    TowingController()
        : ModelPlugin()
    {
    }

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
    {
        // Store model pointer
        model_ = _model;

        // Initialize ROS node
        ros_node_ = gazebo_ros::Node::Get(_sdf);

        // Get parameters
        if (_sdf->HasElement("update_rate"))
            update_rate_ = _sdf->Get<double>("update_rate");

        if (_sdf->HasElement("wheel_separation"))
            wheel_separation_ = _sdf->Get<double>("wheel_separation");

        if (_sdf->HasElement("wheel_diameter"))
            wheel_diameter_ = _sdf->Get<double>("wheel_diameter");

        if (_sdf->HasElement("wheelbase"))
            wheelbase_ = _sdf->Get<double>("wheelbase");

        if (_sdf->HasElement("max_wheel_torque"))
            max_wheel_torque_ = _sdf->Get<double>("max_wheel_torque");

        if (_sdf->HasElement("max_steering_angle"))
            max_steering_angle_ = _sdf->Get<double>("max_steering_angle");

        // Get joint names
        std::string rear_left_joint_name = "rear_left_wheel_joint";
        std::string rear_right_joint_name = "rear_right_wheel_joint";
        std::string steer_joint_name = "steer_joint";

        if (_sdf->HasElement("rear_left_joint"))
            rear_left_joint_name = _sdf->Get<std::string>("rear_left_joint");
        if (_sdf->HasElement("rear_right_joint"))
            rear_right_joint_name = _sdf->Get<std::string>("rear_right_joint");
        if (_sdf->HasElement("steering_joint"))
            steer_joint_name = _sdf->Get<std::string>("steering_joint");

        // Get joints
        rear_left_joint_ = model_->GetJoint(rear_left_joint_name);
        rear_right_joint_ = model_->GetJoint(rear_right_joint_name);
        steer_joint_ = model_->GetJoint(steer_joint_name);

        if (!rear_left_joint_ || !rear_right_joint_ || !steer_joint_) {
            RCLCPP_ERROR(ros_node_->get_logger(), "Failed to get joints!");
            return;
        }

        // Set ODE parameters for force control
        rear_left_joint_->SetParam("fmax", 0, max_wheel_torque_);
        rear_right_joint_->SetParam("fmax", 0, max_wheel_torque_);
        rear_left_joint_->SetParam("vel", 0, 0.0);
        rear_right_joint_->SetParam("vel", 0, 0.0);

        // Subscribe to cmd_vel
        cmd_vel_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&TowingController::cmdVelCallback, this, std::placeholders::_1));

        // Publisher for odometry
        odom_pub_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

        // TF broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(ros_node_);

        // Update connection
        update_connection_ = event::Events::ConnectWorldUpdateBegin(
            std::bind(&TowingController::OnUpdate, this));

        // Initialize odometry
        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;
        last_update_time_ = model_->GetWorld()->SimTime();

        RCLCPP_INFO(ros_node_->get_logger(), "Towing Controller loaded successfully!");
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        target_linear_vel_ = msg->linear.x;
        target_angular_vel_ = msg->angular.z;
    }

    void OnUpdate()
    {
        common::Time current_time = model_->GetWorld()->SimTime();
        double dt = (current_time - last_update_time_).Double();

        if (dt < 1.0 / update_rate_)
            return;

        // Clamp dt to prevent instability
        if (dt > 0.1)
            dt = 0.1;

        // Calculate steering angle from angular velocity (Ackermann)
        double steering_angle = 0.0;
        if (std::abs(target_linear_vel_) > 0.01) {
            // R = v / omega
            // tan(steering_angle) = wheelbase / R
            if (std::abs(target_angular_vel_) > 0.01) {
                double turning_radius = target_linear_vel_ / target_angular_vel_;
                steering_angle = std::atan(wheelbase_ / turning_radius);
            }
        } else if (std::abs(target_angular_vel_) > 0.01) {
            // Rotate in place - use max steering
            steering_angle = target_angular_vel_ > 0 ? max_steering_angle_ : -max_steering_angle_;
        }

        // Clamp steering angle
        steering_angle = std::max(-max_steering_angle_,
            std::min(max_steering_angle_, steering_angle));

        // Set steering joint position with PID control
        double steer_error = steering_angle - steer_joint_->Position(0);
        double steer_torque = 100.0 * steer_error; // Simple P controller for steering
        steer_torque = std::max(-500.0, std::min(500.0, steer_torque));
        steer_joint_->SetForce(0, steer_torque);

        // Calculate wheel velocities for differential drive on rear wheels
        double left_vel = target_linear_vel_;
        double right_vel = target_linear_vel_;

        if (std::abs(steering_angle) > 0.01) {
            // Adjust wheel velocities based on Ackermann geometry
            double turn_radius = wheelbase_ / std::tan(std::abs(steering_angle));

            if (steering_angle > 0) // Turning left
            {
                left_vel = target_linear_vel_ * (turn_radius - wheel_separation_ / 2) / turn_radius;
                right_vel = target_linear_vel_ * (turn_radius + wheel_separation_ / 2) / turn_radius;
            } else // Turning right
            {
                left_vel = target_linear_vel_ * (turn_radius + wheel_separation_ / 2) / turn_radius;
                right_vel = target_linear_vel_ * (turn_radius - wheel_separation_ / 2) / turn_radius;
            }
        }

        // Convert linear velocity to angular velocity for wheels
        double wheel_radius = wheel_diameter_ / 2.0;
        double left_wheel_vel = left_vel / wheel_radius;
        double right_wheel_vel = right_vel / wheel_radius;

        // PID controller for wheel velocity
        double kp = 100.0; // Proportional gain
        double ki = 2.0; // Integral gain
        double kd = 8.0; // Derivative gain

        double left_error = left_wheel_vel - rear_left_joint_->GetVelocity(0);
        double right_error = right_wheel_vel - rear_right_joint_->GetVelocity(0);

        // Update integral (with anti-windup)
        left_integral_ += left_error * dt;
        right_integral_ += right_error * dt;
        left_integral_ = std::max(-100.0, std::min(100.0, left_integral_));
        right_integral_ = std::max(-100.0, std::min(100.0, right_integral_));

        // Calculate derivative
        double left_derivative = (left_error - left_prev_error_) / dt;
        double right_derivative = (right_error - right_prev_error_) / dt;

        // PID output
        double left_torque = kp * left_error + ki * left_integral_ + kd * left_derivative;
        double right_torque = kp * right_error + ki * right_integral_ + kd * right_derivative;

        // Clamp torque
        left_torque = std::max(-max_wheel_torque_, std::min(max_wheel_torque_, left_torque));
        right_torque = std::max(-max_wheel_torque_, std::min(max_wheel_torque_, right_torque));

        // Use velocity control with max force (more reliable in Gazebo ODE)
        rear_left_joint_->SetParam("vel", 0, left_wheel_vel);
        rear_right_joint_->SetParam("vel", 0, right_wheel_vel);
        rear_left_joint_->SetParam("fmax", 0, std::abs(left_torque));
        rear_right_joint_->SetParam("fmax", 0, std::abs(right_torque));

        // Debug logging (setiap 2 detik)
        static double last_log_time = 0;
        if (current_time.Double() - last_log_time > 2.0) {
            RCLCPP_INFO(ros_node_->get_logger(),
                "Target vel: %.2f, Left: err=%.2f torque=%.2f vel=%.2f, Right: err=%.2f torque=%.2f vel=%.2f",
                target_linear_vel_, left_error, left_torque, rear_left_joint_->GetVelocity(0),
                right_error, right_torque, rear_right_joint_->GetVelocity(0));
            last_log_time = current_time.Double();
        }

        // Store error for next iteration
        left_prev_error_ = left_error;
        right_prev_error_ = right_error;

        // Update odometry
        updateOdometry(dt);
        publishOdometry(current_time);

        last_update_time_ = current_time;
    }

    void updateOdometry(double dt)
    {
        // Get actual wheel velocities
        double left_vel = rear_left_joint_->GetVelocity(0) * (wheel_diameter_ / 2.0);
        double right_vel = rear_right_joint_->GetVelocity(0) * (wheel_diameter_ / 2.0);

        // Calculate robot velocity
        double linear_vel = (left_vel + right_vel) / 2.0;
        double angular_vel = (right_vel - left_vel) / wheel_separation_;

        // Update position
        double delta_x = linear_vel * std::cos(theta_) * dt;
        double delta_y = linear_vel * std::sin(theta_) * dt;
        double delta_theta = angular_vel * dt;

        x_ += delta_x;
        y_ += delta_y;
        theta_ += delta_theta;

        // Normalize theta
        theta_ = std::atan2(std::sin(theta_), std::cos(theta_));

        vx_ = linear_vel;
        vy_ = 0.0;
        vth_ = angular_vel;
    }

    void publishOdometry(common::Time current_time)
    {
        // Publish TF
        geometry_msgs::msg::TransformStamped odom_tf;
        odom_tf.header.stamp = rclcpp::Time(current_time.sec, current_time.nsec);
        odom_tf.header.frame_id = "odom";
        odom_tf.child_frame_id = "base_link";

        odom_tf.transform.translation.x = x_;
        odom_tf.transform.translation.y = y_;
        odom_tf.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom_tf.transform.rotation = tf2::toMsg(q);

        tf_broadcaster_->sendTransform(odom_tf);

        // Publish Odometry message
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = rclcpp::Time(current_time.sec, current_time.nsec);
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = tf2::toMsg(q);

        odom.twist.twist.linear.x = vx_;
        odom.twist.twist.linear.y = vy_;
        odom.twist.twist.angular.z = vth_;

        odom_pub_->publish(odom);
    }

    // Model and joints
    physics::ModelPtr model_;
    physics::JointPtr rear_left_joint_;
    physics::JointPtr rear_right_joint_;
    physics::JointPtr steer_joint_;

    // ROS
    gazebo_ros::Node::SharedPtr ros_node_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Gazebo connection
    event::ConnectionPtr update_connection_;

    // Parameters
    double update_rate_ = 50.0;
    double wheel_separation_ = 0.86;
    double wheel_diameter_ = 0.46;
    double wheelbase_ = 1.011; // Distance from rear axle to steer joint
    double max_wheel_torque_ = 3000.0;
    double max_steering_angle_ = 1.8;

    // Command
    double target_linear_vel_ = 0.0;
    double target_angular_vel_ = 0.0;

    // PID error tracking
    double left_integral_ = 0.0;
    double right_integral_ = 0.0;
    double left_prev_error_ = 0.0;
    double right_prev_error_ = 0.0;

    // Odometry
    double x_ = 0.0;
    double y_ = 0.0;
    double theta_ = 0.0;
    double vx_ = 0.0;
    double vy_ = 0.0;
    double vth_ = 0.0;
    common::Time last_update_time_;
};

GZ_REGISTER_MODEL_PLUGIN(TowingController)
}
