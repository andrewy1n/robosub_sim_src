#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <gz/transport/Node.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

#include <gz/msgs/quaternion.pb.h>
#include <gz/msgs/wrench.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_wrench.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/pose.pb.h>

#include <cmath>

class WrenchMovement : public rclcpp::Node {
	public:
		WrenchMovement()
		: Node("wrench_movement_node"),
			force_magnitude_(100000.0),
			torque_magnitude_(5000.0),

			drag_coefficient_(100000.0),
			rotation_damping_(8000.0)
		{
			prev_time_ = this->now();
			gz_node_ = std::make_shared<gz::transport::Node>();

			thrust_force_.Set(0.0, 0.0, 0.0);
			thrust_torque_.Set(0.0, 0.0, 0.0);
			drag_force_.Set(0.0, 0.0, 0.0);
			drag_torque_.Set(0.0, 0.0, 0.0);

			control_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
				"/cmd_vel", 10,
				std::bind(&WrenchMovement::control_callback, this, std::placeholders::_1)
			);

			timer_ = this->create_wall_timer(
				std::chrono::milliseconds(100),
				std::bind(&WrenchMovement::timer_callback, this)
			);
			
			
			bool ok = gz_node_->Subscribe("/model/high_level_robosub/pose", &WrenchMovement::pose_callback, this);
			if (!ok) {
				RCLCPP_ERROR(this->get_logger(), "Failed to subscribe to pose topic.");
			}
			
			gz_node_pub_ = gz_node_->Advertise<gz::msgs::EntityWrench>("/world/pool/wrench");
			if (!gz_node_pub_) {
				RCLCPP_ERROR(this->get_logger(), "Failed to advertise on /world/pool/wrench");
			}	
		}

	private:
		void pose_callback(const gz::msgs::Pose_V &msg) { // rotation stabilization and drag force
			rclcpp::Time current_time = this->now();
			double dt = (current_time - prev_time_).seconds();

			if (!first_time_ && dt > 0.0) {
				gz::msgs::Pose pose = msg.pose(0);
				gz::msgs::Vector3d position_msg = pose.position();
				gz::msgs::Quaternion orientation_msg = pose.orientation();

				gz::math::Vector3d position(position_msg.x(), position_msg.y(), position_msg.z());
				gz::math::Quaterniond orientation(orientation_msg.w(), orientation_msg.x(), orientation_msg.y(), orientation_msg.z());
				
				// Rotation dampening
				{
					gz::math::Quaterniond q_delta = orientation * prev_orientation_.Inverse();

					gz::math::Vector3d axis;
					double angle;
					q_delta.AxisAngle(axis, angle);

					gz::math::Vector3d angular_velocity = axis * (angle / dt);
					drag_torque_ = -rotation_damping_ * angular_velocity;
				}

				// Translational drag force
				{
					gz::math::Vector3d velocity = (position - prev_position_) / dt;
					drag_force_ = -drag_coefficient_ * velocity;
				}

				// Update for next time
				prev_orientation_ = orientation;
				prev_position_ = position;
				prev_time_ = current_time;
			} else {
				gz::msgs::Pose pose = msg.pose(0);
				gz::msgs::Quaternion orientation = pose.orientation();
				prev_orientation_ = gz::math::Quaterniond(orientation.w(), orientation.x(), orientation.y(), orientation.z());
				prev_time_ = current_time;
				first_time_ = false;
			}
		}

		void control_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
			gz::math::Vector3d lookVector = 	prev_orientation_.RotateVector(gz::math::Vector3d(1, 0, 0));
			gz::math::Vector3d rightVector = 	prev_orientation_.RotateVector(gz::math::Vector3d(0, 1, 0));
			gz::math::Vector3d upVector = 		prev_orientation_.RotateVector(gz::math::Vector3d(0, 0, 1));
			//RCLCPP_INFO(this->get_logger(), "Orientation - %.2f, %.2f, %.2f, %.2f \n", prev_orientation_.W(), prev_orientation_.X(), prev_orientation_.Y(), prev_orientation_.Z());
			//RCLCPP_INFO(this->get_logger(), "LookVector - %.2f, %.2f, %.2f \n", lookVector.X(), lookVector.Y(), lookVector.Z());
			//RCLCPP_INFO(this->get_logger(), "Control - linear: %.2f, %.2f, %.2f, angular: %.2f, %.2f, %.2f \n", 
			//	msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z);
			
			thrust_force_ = (
				lookVector 	* msg->linear.x * force_magnitude_ + 
				rightVector * msg->linear.y * force_magnitude_ +
				upVector 	* msg->linear.z * force_magnitude_
			);

			thrust_torque_ = (
				lookVector 	* msg->angular.x * torque_magnitude_ +
				rightVector * msg->angular.y * torque_magnitude_ +
				upVector 	* msg->angular.z * torque_magnitude_
			);

		}

		void timer_callback() {
			gz::msgs::EntityWrench msg;

			gz::msgs::Wrench wrench;
			wrench.mutable_force()->set_x(thrust_force_.X() + drag_force_.X());
			wrench.mutable_force()->set_y(thrust_force_.Y() + drag_force_.Y());
			wrench.mutable_force()->set_z(thrust_force_.Z() + drag_force_.Z());
			wrench.mutable_torque()->set_x(thrust_torque_.X() + drag_torque_.X());
			wrench.mutable_torque()->set_y(thrust_torque_.Y() + drag_torque_.Y());
			wrench.mutable_torque()->set_z(thrust_torque_.Z() + drag_torque_.Z());

			msg.mutable_wrench()->CopyFrom(wrench);
			msg.mutable_entity()->set_name("high_level_robosub");
			msg.mutable_entity()->set_type(gz::msgs::Entity::MODEL);

			if (!(gz_node_pub_.Publish(msg))) {
				RCLCPP_ERROR(this->get_logger(), "Failed to publish wrench message.");
			}
		}

		gz::math::Vector3d thrust_force_;
		gz::math::Vector3d thrust_torque_;
		gz::math::Vector3d drag_force_;
		gz::math::Vector3d drag_torque_;

		double force_magnitude_;
		double torque_magnitude_;
		double drag_coefficient_;
		double rotation_damping_;

		gz::math::Vector3d prev_position_;
		gz::math::Quaterniond prev_orientation_;
		rclcpp::Time prev_time_;
		bool first_time_ = true;

		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr control_sub_;

		std::shared_ptr<gz::transport::Node> gz_node_;
		gz::transport::Node::Publisher gz_node_pub_;
};

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<WrenchMovement>();

	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(node);
	executor.spin();

	rclcpp::shutdown();
	return 0;
}
