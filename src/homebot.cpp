#include <chrono>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <array>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <boost/asio.hpp>

#define head1 0xAA
#define head2 0x55
#define sendType_velocity 0x11
#define sendType_pid 0x12
#define sendType_params 0x13

using namespace std;
using namespace boost::asio;
using namespace std::chrono_literals;

io_service iosev;
serial_port sp(iosev);

uint8_t checksum(uint8_t *buf, size_t len)
{
    uint8_t sum = 0x00;
    for (size_t i = 0; i < len; i++)
    {
        sum += *(buf + i);
    }
    return sum;
}

// PID parameter sending function
void SetPID(int p, int i, int d)
{
    static uint8_t tmp[11];
    tmp[0] = head1;
    tmp[1] = head2;
    tmp[2] = 0x0b;
    tmp[3] = sendType_pid;
    tmp[4] = (p >> 8) & 0xff;
    tmp[5] = p & 0xff;
    tmp[6] = (i >> 8) & 0xff;
    tmp[7] = i & 0xff;
    tmp[8] = (d >> 8) & 0xff;
    tmp[9] = d & 0xff;
    tmp[10] = checksum(tmp, 10);
    write(sp, buffer(tmp, 11));
}

// Robot parameter sending function
void SetParams(double linear_correction, double angular_correction)
{
    static uint8_t tmp[9];
    tmp[0] = head1;
    tmp[1] = head2;
    tmp[2] = 0x09;
    tmp[3] = sendType_params;
    tmp[4] = (int16_t)((int16_t)(linear_correction * 1000) >> 8) & 0xff;
    tmp[5] = (int16_t)(linear_correction * 1000) & 0xff;
    tmp[6] = (int16_t)((int16_t)(angular_correction * 1000) >> 8) & 0xff;
    tmp[7] = (int16_t)(angular_correction * 1000) & 0xff;
    tmp[8] = checksum(tmp, 8);
    write(sp, buffer(tmp, 9));
}

// Robot speed transmission function
void SetVelocity(double x, double y, double yaw)
{
    static uint8_t tmp[11];
    tmp[0] = head1;
    tmp[1] = head2;
    tmp[2] = 0x0b;
    tmp[3] = sendType_velocity;
    tmp[4] = ((int16_t)(x * 1000) >> 8) & 0xff;
    tmp[5] = ((int16_t)(x * 1000)) & 0xff;
    tmp[6] = ((int16_t)(y * 1000) >> 8) & 0xff;
    tmp[7] = ((int16_t)(y * 1000)) & 0xff;
    tmp[8] = ((int16_t)(yaw * 1000) >> 8) & 0xff;
    tmp[9] = ((int16_t)(yaw * 1000)) & 0xff;
    tmp[10] = checksum(tmp, 10);
    write(sp, buffer(tmp, 11));
}

class HomeBotNode : public rclcpp::Node
{
public:
    HomeBotNode()
        : Node("homebot"), cmd_time(this->now()), x(0.0), y(0.0), yaw(0.0)
    {
        // Declare and get parameters
        this->declare_parameter<std::string>("port_name", "/dev/ttyACM0");
        this->declare_parameter<int>("baud_rate", 115200);
        this->declare_parameter<double>("linear_correction", 1.0);
        this->declare_parameter<double>("angular_correction", 1.0);
        this->declare_parameter<bool>("publish_odom_transform", true);

        port_name = this->get_parameter("port_name").as_string();
        baud_rate = this->get_parameter("baud_rate").as_int();
        linear_correction = this->get_parameter("linear_correction").as_double();
        angular_correction = this->get_parameter("angular_correction").as_double();
        publish_odom_transform = this->get_parameter("publish_odom_transform").as_bool();

        // Set up serial port
        boost::system::error_code ec;
        sp.open(port_name, ec);
        if (ec)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", ec.message().c_str());
            rclcpp::shutdown();
            return;
        }
        sp.set_option(serial_port::baud_rate(baud_rate));
        sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
        sp.set_option(serial_port::parity(serial_port::parity::none));
        sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
        sp.set_option(serial_port::character_size(8));

        // Set up publishers
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        lvel_pub_ = this->create_publisher<std_msgs::msg::Int32>("motor/lvel", 10);
        rvel_pub_ = this->create_publisher<std_msgs::msg::Int32>("motor/rvel", 10);
        lset_pub_ = this->create_publisher<std_msgs::msg::Int32>("motor/lset", 10);
        rset_pub_ = this->create_publisher<std_msgs::msg::Int32>("motor/rset", 10);

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Set up cmd_vel subscriber
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg)
            {
                x = msg->linear.x;
                y = msg->linear.x;
                yaw = msg->angular.z;
                cmd_time = this->now();
            });

        // Timer to periodically send velocity to robot
        timer_ = this->create_wall_timer(
            20ms, [this]()
            {
                auto current_time = this->now();
                if ((current_time - cmd_time).seconds() > 1)
                {
                    x = 0.0;
                    y = 0.0;
                    yaw = 0.0;
                }
                SetVelocity(x, y, yaw);
            });

        // PID parameters as ROS 2 parameters (can be changed at runtime)
        this->declare_parameter<int>("kp", 0);
        this->declare_parameter<int>("ki", 0);
        this->declare_parameter<int>("kd", 0);

        using std::placeholders::_1;
        pid_param_cb_handle_ = this->add_on_set_parameters_callback(
            std::bind(&HomeBotNode::parametersCallback, this, _1)
        );

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        SetParams(linear_correction, angular_correction);

        // Launch serial reading in a thread
        serial_thread_ = std::thread([this]() { serial_task(); });
    }

    ~HomeBotNode()
    {
        if (serial_thread_.joinable())
            serial_thread_.join();
    }

private:
    // State variables (now class members)
    rclcpp::Time cmd_time;
    double x, y, yaw;

    // Parameters
    std::string port_name;
    int baud_rate;
    bool publish_odom_transform;
    int kp, ki, kd;
    double linear_correction, angular_correction;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr lvel_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr rvel_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr lset_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr rset_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr pid_param_cb_handle_;
    std::thread serial_thread_;

    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &params) {
        for (const auto &param : params) {
            if (param.get_name() == "kp")
                kp = param.as_int();
            if (param.get_name() == "ki")
                ki = param.as_int();
            if (param.get_name() == "kd")
                kd = param.as_int();
        }
        SetPID(kp, ki, kd);
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "";
        return result;
    }

    void serial_task()
    {
        enum frameState
        {
            State_Head1, State_Head2, State_Size, State_Data, State_CheckSum, State_Handle
        };
        frameState state = State_Head1;

        uint8_t frame_size, frame_sum, frame_type;
        uint8_t data[50];

        double imu_list[9];
        double odom_list[6];
        rclcpp::Time now_time, last_time = this->now();

        // Declare all variables here to avoid jump-over initialization
        sensor_msgs::msg::Imu imu_msg;
        geometry_msgs::msg::TransformStamped odom_trans;
        tf2::Quaternion q;
        tf2::Quaternion odom_q;
        nav_msgs::msg::Odometry odom_msg;
        double dt = 0;
        std::array<double, 36> twist_cov = {1e-9, 0, 0, 0, 0, 0,
                                            0, 1e-3, 1e-9, 0, 0, 0,
                                            0, 0, 1e6, 0, 0, 0,
                                            0, 0, 0, 1e6, 0, 0,
                                            0, 0, 0, 0, 1e6, 0,
                                            0, 0, 0, 0, 0, 0.1};
        std::array<double, 36> pose_cov = {1e-9, 0, 0, 0, 0, 0,
                                           0, 1e-3, 1e-9, 0, 0, 0,
                                           0, 0, 1e6, 0, 0, 0,
                                           0, 0, 0, 1e6, 0, 0,
                                           0, 0, 0, 0, 1e6, 0,
                                           0, 0, 0, 0, 0, 1e3};
        std::array<double, 9> o_cov = {1e6, 0, 0, 0, 1e6, 0, 0, 0, 0.05};
        std::array<double, 9> av_cov = {1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e6};
        std::array<double, 9> la_cov = {1e-2, 0, 0, 0, 0, 0, 0, 0, 0};
        std_msgs::msg::Int32 lvel_msg;
        std_msgs::msg::Int32 rvel_msg;
        std_msgs::msg::Int32 lset_msg;
        std_msgs::msg::Int32 rset_msg;

        RCLCPP_INFO(this->get_logger(), "start receive message");
        while (rclcpp::ok())
        {
            switch (state)
            {
            case State_Head1:
                frame_sum = 0x00;
                read(sp, buffer(&data[0], 1));
                state = (data[0] == head1 ? State_Head2 : State_Head1);
                break;
            case State_Head2:
                read(sp, buffer(&data[1], 1));
                state = (data[1] == head2 ? State_Size : State_Head1);
                break;
            case State_Size:
                read(sp, buffer(&data[2], 1));
                frame_size = data[2];
                state = State_Data;
                break;
            case State_Data:
                read(sp, buffer(&data[3], frame_size - 4));
                frame_type = data[3];
                (void)frame_type;
                state = State_CheckSum;
                break;
            case State_CheckSum:
                read(sp, buffer(&data[frame_size - 1], 1));
                frame_sum = checksum(data, frame_size - 1);
                state = (data[frame_size - 1] == frame_sum) ? State_Handle : State_Head1;
                break;
            case State_Handle:
                now_time = this->now();

                // Gyro
                imu_list[0] = ((double)((int16_t)(data[4] * 256 + data[5])) / 32768 * 2000 / 180 * 3.1415);
                imu_list[1] = ((double)((int16_t)(data[6] * 256 + data[7])) / 32768 * 2000 / 180 * 3.1415);
                imu_list[2] = ((double)((int16_t)(data[8] * 256 + data[9])) / 32768 * 2000 / 180 * 3.1415);
                // Acc
                imu_list[3] = ((double)((int16_t)(data[10] * 256 + data[11])) / 32768 * 2 * 9.8);
                imu_list[4] = ((double)((int16_t)(data[12] * 256 + data[13])) / 32768 * 2 * 9.8);
                imu_list[5] = ((double)((int16_t)(data[14] * 256 + data[15])) / 32768 * 2 * 9.8);
                // Angle
                imu_list[6] = ((double)((int16_t)(data[16] * 256 + data[17])) / 10.0);
                imu_list[7] = ((double)((int16_t)(data[18] * 256 + data[19])) / 10.0);
                imu_list[8] = ((double)((int16_t)(data[20] * 256 + data[21])) / 10.0);

                // Publish the IMU message
                imu_msg.header.stamp = now_time;
                imu_msg.header.frame_id = "base_imu_link";
                imu_msg.angular_velocity.x = imu_list[0];
                imu_msg.angular_velocity.y = imu_list[1];
                imu_msg.angular_velocity.z = imu_list[2];
                imu_msg.linear_acceleration.x = imu_list[3];
                imu_msg.linear_acceleration.y = imu_list[4];
                imu_msg.linear_acceleration.z = imu_list[5];

                // Orientation as yaw only (manual conversion)
                q.setRPY(0, 0, imu_list[8] / 180.0 * 3.1415926);
                imu_msg.orientation.x = q.x();
                imu_msg.orientation.y = q.y();
                imu_msg.orientation.z = q.z();
                imu_msg.orientation.w = q.w();

                std::copy(o_cov.begin(), o_cov.end(), imu_msg.orientation_covariance.begin());
                std::copy(av_cov.begin(), av_cov.end(), imu_msg.angular_velocity_covariance.begin());
                std::copy(la_cov.begin(), la_cov.end(), imu_msg.linear_acceleration_covariance.begin());

                imu_pub_->publish(imu_msg);

                // Odom
                odom_list[0] = ((double)((int16_t)(data[22] * 256 + data[23])) / 1000);
                odom_list[1] = ((double)((int16_t)(data[24] * 256 + data[25])) / 1000);
                odom_list[2] = ((double)((int16_t)(data[26] * 256 + data[27])) / 1000);
                odom_list[3] = ((double)((int16_t)(data[28] * 256 + data[29])) / 1000);
                odom_list[4] = ((double)((int16_t)(data[30] * 256 + data[31])) / 1000);
                odom_list[5] = ((double)((int16_t)(data[32] * 256 + data[33])) / 1000);

                // Publish TF
                odom_trans.header.stamp = now_time;
                odom_trans.header.frame_id = "odom";
                odom_trans.child_frame_id = "base_footprint";
                odom_trans.transform.translation.x = odom_list[0];
                odom_trans.transform.translation.y = odom_list[1];
                odom_trans.transform.translation.z = 0.0;

                odom_q.setRPY(0, 0, odom_list[2]);
                odom_trans.transform.rotation.x = odom_q.x();
                odom_trans.transform.rotation.y = odom_q.y();
                odom_trans.transform.rotation.z = odom_q.z();
                odom_trans.transform.rotation.w = odom_q.w();

                if (publish_odom_transform)
                    tf_broadcaster_->sendTransform(odom_trans);

                // Publish odometry message
                odom_msg.header.stamp = now_time;
                odom_msg.header.frame_id = "odom";
                odom_msg.pose.pose.position.x = odom_list[0];
                odom_msg.pose.pose.position.y = odom_list[1];
                odom_msg.pose.pose.position.z = 0.0;
                odom_msg.pose.pose.orientation.x = odom_q.x();
                odom_msg.pose.pose.orientation.y = odom_q.y();
                odom_msg.pose.pose.orientation.z = odom_q.z();
                odom_msg.pose.pose.orientation.w = odom_q.w();
                odom_msg.child_frame_id = "base_footprint";
                dt = (now_time - last_time).seconds();
                if (dt < 1e-6)
                    dt = 1e-6;
                odom_msg.twist.twist.linear.x = odom_list[3] / dt;
                odom_msg.twist.twist.linear.y = odom_list[4] / dt;
                odom_msg.twist.twist.angular.z = odom_list[5] / dt;

                std::copy(twist_cov.begin(), twist_cov.end(), odom_msg.twist.covariance.begin());
                std::copy(pose_cov.begin(), pose_cov.end(), odom_msg.pose.covariance.begin());

                odom_pub_->publish(odom_msg);

                // Motor messages
                lvel_msg.data = ((int16_t)(data[34] * 256 + data[35]));
                rvel_msg.data = ((int16_t)(data[36] * 256 + data[37]));
                lset_msg.data = ((int16_t)(data[38] * 256 + data[39]));
                rset_msg.data = ((int16_t)(data[40] * 256 + data[41]));
                lvel_pub_->publish(lvel_msg);
                rvel_pub_->publish(rvel_msg);
                lset_pub_->publish(lset_msg);
                rset_pub_->publish(rset_msg);

                last_time = now_time;
                state = State_Head1;
                break;
            default:
                state = State_Head1;
                break;
            }
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HomeBotNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

