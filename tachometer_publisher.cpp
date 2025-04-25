#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "jetgpio.h"
#include <chrono>
#include <cmath>
#include <deque>
#include <algorithm>

#define LEFT_TACH_GPIO 33 // 33
#define RIGHT_TACH_GPIO 31 // 31
#define PPR_LEFT 4
#define PPR_RIGHT 8
#define DEBOUNCE_US 1000 // 1 ms debounce interval

class TachometerOdomNode : public rclcpp::Node
{
public:
    TachometerOdomNode()
    : Node("tachometer_odom_publisher"), left_pulse_count_(0), right_pulse_count_(0), x_(0.0), y_(0.0), theta_(0.0)
    {
        if (gpioInitialise() < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize JetGPIO!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "JetGPIO initialized");

        gpioSetMode(LEFT_TACH_GPIO, JET_INPUT);
        gpioSetMode(RIGHT_TACH_GPIO, JET_INPUT);

        instance_ = this;

        left_timestamp_ = 0;
        right_timestamp_ = 0;
        last_left_tick_ = std::chrono::steady_clock::now();
        last_right_tick_ = std::chrono::steady_clock::now();

        gpioSetISRFunc(LEFT_TACH_GPIO, RISING_EDGE,1000, &left_timestamp_, left_pulse_callback);
        gpioSetISRFunc(RIGHT_TACH_GPIO, RISING_EDGE,1000, &right_timestamp_, right_pulse_callback);

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/wheel/odom", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        last_time_ = this->get_clock()->now();

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&TachometerOdomNode::publish_odom, this));

      	cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
	    "/cmd_vel", 10, std::bind(&TachometerOdomNode::cmd_vel_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Tachometer Odom Node started");
    }

    ~TachometerOdomNode()
    {
        gpioTerminate();
    }

private:
    static TachometerOdomNode* instance_;
    volatile int left_pulse_count_;
    volatile int right_pulse_count_;

    std::chrono::steady_clock::time_point last_left_tick_;
    std::chrono::steady_clock::time_point last_right_tick_;
    unsigned long left_timestamp_;
    unsigned long right_timestamp_;
    double dx = 0.0;
    double dy = 0.0;
    double v = 0.0;
    double x_, y_, theta_, total_theta;
    double total_distance_ = 0.0;
    double total_left = 0.0;

    const double wheel_radius_ = 0.0525;   // meters 0.0525 originally
    const double wheel_base_ = 0.33337;    // meters
    const float max_steering_angle_ = 0.785; // ±30 deg in radians now 45 deg 

    float last_servo_pwm_ = 0.0f; // initial mid value
    double steering_angle_rad_ = 0.0;

    std::deque<double> left_rpm_history_;
    std::deque<double> right_rpm_history_;
    const size_t rpm_window_size_ = 5;

    rclcpp::Time last_time_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    static void left_pulse_callback()
    {
        if (!instance_) return;
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - instance_->last_left_tick_).count();
        if (duration > DEBOUNCE_US)
        {
            instance_->last_left_tick_ = now;
            instance_->left_pulse_count_++;
        }
    }

    static void right_pulse_callback()
    {
        if (!instance_) return;
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - instance_->last_right_tick_).count();
        if (duration > DEBOUNCE_US)
        {
            instance_->last_right_tick_ = now;
            instance_->right_pulse_count_++;
        }
    }
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
	//RCLCPP_INFO(this->get_logger(), "Linear: x=%.2f, y=%.2f | Angular: z=%.2f", msg->linear.x, msg->linear.y, msg->angular.z);
	last_servo_pwm_ = msg->angular.z;
	steering_angle_rad_ = last_servo_pwm_  * max_steering_angle_;
	RCLCPP_INFO(this->get_logger(), "steering angle = %.2f", steering_angle_rad_);
    }
    double compute_moving_average(std::deque<double>& history, double new_value)
    {
        history.push_back(new_value);
        if (history.size() > rpm_window_size_)
            history.pop_front();

        double sum = 0.0;
        for (double val : history)
            sum += val;
        return sum / static_cast<double>(history.size());
    }
    void publish_odom()
    {
        auto now = this->get_clock()->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        double left_rps = static_cast<double>(left_pulse_count_) / PPR_LEFT / dt;
        double right_rps = static_cast<double>(right_pulse_count_) / PPR_RIGHT / dt;
	//RCLCPP_INFO(this->get_logger(), "left %.2d, and right %.2d", left_pulse_count_, right_pulse_count_);

        double left_rpm = compute_moving_average(left_rpm_history_, left_rps * 60.0);
        double right_rpm = compute_moving_average(right_rpm_history_, right_rps * 60.0);

        double v_left = 2 * M_PI * wheel_radius_ * (left_rpm / 60.0);
        double v_right = 2 * M_PI * wheel_radius_ * (right_rpm / 60.0);
	if (v_left < 270)
	{
		v = v_right;
	}else {
		v = v_left;
	}
	if (steering_angle_rad_ != 0)
	{
		double turning_radius = wheel_base_ / std::tan(steering_angle_rad_);
		double distance = v * dt;
		theta_ = distance / turning_radius; // negative?
		if (v != 0)
		{
			dx = turning_radius * std::cos(theta_);
			dy = turning_radius *  (1 - std::sin(theta_));
		} else {
			dx = 0.0;
			dy = 0.0;
		}
	}else {
		theta_ = 0.0;
		dx = v * dt;
		dy = 0.0;
	}
	// send x, y, theta = get_current_position()
	x_ += dx/4;
	y_ += dy/4;
	total_theta += theta_;
        RCLCPP_INFO(this->get_logger(), "distance travelled x %.2f and y %.2f dy %0.2f total theta %0.2f", x_, y_, dy, total_theta);
	
	double cov = 0.005;
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = now;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, total_theta);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        odom_msg.twist.twist.linear.x = v;
	odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.angular.z = theta_ / dt;

        odom_msg.pose.covariance = {
            cov * std::abs(x_) , 0,    0,    0,    0,    0,
            0,    cov * std::abs(y_), 0,    0,    0,    0,
            0,    0,    cov,    0,    0,    0,
            0,    0,    0,    cov,    0,    0,
            0,    0,    0,    0,    cov,    0,
            0,    0,    0,    0,    0,    cov * std::abs(total_theta)
        };
	odom_msg.twist.covariance = {
	    cov * std::abs(v), 0, 0, 0, 0, 0,
	    0, cov, 0, 0, 0, 0,
	    0, 0, cov, 0, 0, 0,
	    0, 0, 0, cov, 0, 0,
	    0, 0, 0, 0, cov, 0,
	    0, 0, 0, 0, 0, cov * std::abs(theta_ / dt)
	};
        odom_pub_->publish(odom_msg);

        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = now;
        tf_msg.header.frame_id = "odom";
        tf_msg.child_frame_id = "base_link";
        tf_msg.transform.translation.x = x_;
        tf_msg.transform.translation.y = y_;
        tf_msg.transform.translation.z = 0.0;
        tf_msg.transform.rotation.x = q.x();
        tf_msg.transform.rotation.y = q.y();
        tf_msg.transform.rotation.z = q.z();
        tf_msg.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(tf_msg);

        left_pulse_count_ = 0;
        right_pulse_count_ = 0;
    }
};

TachometerOdomNode* TachometerOdomNode::instance_ = nullptr;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TachometerOdomNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
