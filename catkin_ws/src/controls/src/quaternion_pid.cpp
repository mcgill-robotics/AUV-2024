#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Matrix3x3.h>

double Kp, Ki, Kd, windup_limit;
tf2::Quaternion body_quat, goal_quat;
bool has_goal = false;
geometry_msgs::Vector3 angular_velocity;
bool enabled;
ros::Time previous_time;
tf2::Vector3 torque_integral;

ros::Subscriber pose_sub, angular_velocity_sub, goal_sub, enable_sub;
ros::Publisher pub_roll, pub_pitch, pub_yaw, pub_error_quat;

void set_pose(const geometry_msgs::Pose::ConstPtr& msg) {
    body_quat = tf2::Quaternion(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    if (body_quat.w() < 0) body_quat = -body_quat;
}

void set_ang_vel(const geometry_msgs::Vector3::ConstPtr& msg) {
    angular_velocity = *msg;
}

void set_goal(const geometry_msgs::Quaternion::ConstPtr& msg) {
    goal_quat = tf2::Quaternion(msg->w, msg->x, msg->y, msg->z);
    if (goal_quat.w() < 0) goal_quat = -goal_quat;
    torque_integral = tf2::Vector3(0, 0, 0);
    previous_time = ros::Time::now();
    has_goal = true;
}

void set_enabled(const std_msgs::Bool::ConstPtr& msg) {
    enabled = msg->data;
}

tf2::Quaternion calculateQuatError(const tf2::Quaternion& q1, const tf2::Quaternion& q2) {
    return q1.inverse() * q2;
}

tf2::Vector3 controlEffort() {
    tf2::Quaternion error_quat = calculateQuatError(body_quat, goal_quat);
    pub_error_quat.publish(error_quat.w());

    if (error_quat.w() < 0) error_quat = -error_quat;

    ros::Duration delta_t = ros::Time::now() - previous_time;
    previous_time = ros::Time::now();
    tf2::Vector3 axis = tf2::Vector3(error_quat.x(), error_quat.y(), error_quat.z());
    tf2::Vector3 diff = axis * delta_t.toSec();
    torque_integral += diff;

    double torque_integral_norm = torque_integral.length();
    if (torque_integral_norm > windup_limit) {
        torque_integral = torque_integral * (windup_limit / torque_integral_norm);
    }

    tf2::Vector3 proportional_effort(Kp * error_quat.x(), Kp * error_quat.y(), Kp * error_quat.z());
    tf2::Vector3 derivative_effort = Kd * tf2::Vector3(angular_velocity.x, angular_velocity.y, angular_velocity.z);
    tf2::Vector3 integral_effort = Ki * torque_integral;

    tf2::Vector3 control_effort = proportional_effort - derivative_effort + integral_effort;

    // tf2::Matrix3x3 inertial_matrix(0.1376267915, 0, 0,
    //                                0, 0.6490918050833332, 0,
    //                                0, 0, 0.6490918050833332);

    // tf2::Vector3 torque = inertial_matrix * control_effort;
    return control_effort;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "quaternion_pid");
    ros::NodeHandle nh("~");

    nh.param("~Kp", Kp, 1.0);
    nh.param("~Ki", Ki, 0.0);
    nh.param("~Kd", Kd, 0.0);
    nh.param("~windup_limit", windup_limit, 1.0);

    pose_sub = nh.subscribe("/state/pose", 1, set_pose);
    angular_velocity_sub = nh.subscribe("/state/angular_velocity", 1, set_ang_vel);
    goal_sub = nh.subscribe("/controls/pid/quat/setpoint", 1, set_goal);
    enable_sub = nh.subscribe("/controls/pid/quat/enable", 1, set_enabled);

    pub_roll = nh.advertise<std_msgs::Float64>("/controls/torque/roll", 1);
    pub_pitch = nh.advertise<std_msgs::Float64>("/controls/torque/pitch", 1);
    pub_yaw = nh.advertise<std_msgs::Float64>("/controls/torque/yaw", 1);
    pub_error_quat = nh.advertise<std_msgs::Float64>("/controls/pid/quat/error", 1);

    ros::Rate rate(100);
    while (ros::ok()) {
        if (enabled && has_goal) {
            Kp = 1.0;
            Ki = 0.0;
            Kd = 0.0;
            windup_limit = 1.0;

            tf2::Vector3 torque = controlEffort();
            std_msgs::Float64 roll_effort, pitch_effort, yaw_effort;
            roll_effort.data = torque.x();
            pitch_effort.data = torque.y();
            yaw_effort.data = torque.z();

            pub_roll.publish(roll_effort);
            pub_pitch.publish(pitch_effort);
            pub_yaw.publish(yaw_effort);
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}