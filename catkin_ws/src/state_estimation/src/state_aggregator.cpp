#include <ros/ros.h>
#include "sensor.h"
#include "dvl.h"
#include "imu.h"
#include "depth.h"
#include <geometry_msgs/Pose.h>
#include <sstream>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>
#include <iostream>

bool update_state_on_clock;
ros::Time last_clock_msg;
Sensor* depth_estimators[1];
ros::Publisher pub_pose;
ros::Publisher pub_x;
ros::Publisher pub_y;
ros::Publisher pub_z;
ros::Publisher pub_av;
ros::Publisher pub_imu_status;
ros::Publisher pub_dvl_status;
ros::Publisher pub_depth_status;
DepthSensor* depth;

void update_state(const ros::TimerEvent& event) {
    if(update_state_on_clock) {
        if(event.current_expected == last_clock_msg) {
            return;
        }
        last_clock_msg.sec = event.current_expected.sec;
        last_clock_msg.nsec = event.current_expected.nsec;
    }
    std_msgs::Bool depth_status_msg;
    depth_status_msg.data = depth->is_active();
    pub_depth_status.publish(depth_status_msg);

    double* z = NULL;
    for(Sensor* sensor : depth_estimators) {
        ROS_DEBUG("%s",sensor->sensor_name.c_str());
        if(sensor->is_active()) {
            z = new double(sensor->depth);
            break;
        }
    }
    if(z != NULL) {
        std_msgs::Float64 depth_msg;
        depth_msg.data = *z;
        pub_z.publish(depth_msg);
        delete z;
    }
}

int main(int argc, char **argv) {
    ros::init(argc,argv,"state_aggregator");
    ros::NodeHandle n;

    // if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    //     ros::console::notifyLoggerLevelsChanged();
    // }


    depth = new DepthSensor(0.0,n,std::string("depth"));
    depth_estimators[0] = depth;

    pub_pose = n.advertise<geometry_msgs::Pose>("/state/pose",1);
    pub_x = n.advertise<std_msgs::Float64>("/state/x",1);
    pub_y = n.advertise<std_msgs::Float64>("/state/y",1);
    pub_z = n.advertise<std_msgs::Float64>("/state/z",1);
    pub_av = n.advertise<geometry_msgs::Vector3>("/state/angular_velocity",1);
    pub_imu_status = n.advertise<std_msgs::Bool>("/sensors/imu/status",1);
    pub_dvl_status = n.advertise<std_msgs::Bool>("/sensors/dvl/status",1);
    pub_depth_status = n.advertise<std_msgs::Bool>("/sensors/depth/status",1);


    if(!n.getParam("update_state_on_clock",update_state_on_clock)) {
        ROS_ERROR("Failed to get param 'update_state_on_clock'");
    }

    int update_rate;
    if(!n.getParam("update_rate",update_rate)) {
        ROS_ERROR("Failed to get param 'update_rate'");
    }
    double freq = 1.0/update_rate;

    ros::Timer timer = n.createTimer(ros::Duration(freq), update_state);

    ros::spin();
    delete depth;
    return 0;
}