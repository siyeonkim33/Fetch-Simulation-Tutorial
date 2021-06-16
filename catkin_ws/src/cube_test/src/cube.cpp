#include <iostream>

#include "ros/ros.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TransformStamped.h"

#include <actionlib/client/simple_action_client.h>
#include "control_msgs/PointHeadAction.h"
#include "control_msgs/PointHeadGoal.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf/transform_listener.h>
#include <tf2_ros/buffer_client.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <pcl_conversions/pcl_conversions.h>
// #include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>

#include <sensor_msgs/PointCloud2.h>

#include <typeinfo>

using namespace std;
const int PI = 3.141592;

typedef actionlib::SimpleActionClient<control_msgs::PointHeadAction> PointHeadClient;
ros::Publisher pub;
ros::Subscriber sub;

// Point the head using controller
class HeadAction{
    private:
        PointHeadClient* client;

    public:
        HeadAction(){
            client = new PointHeadClient("head_controller/point_head", true);
            ROS_INFO("Waiting for head_controller...");
            client->waitForServer(ros::Duration(5.0));
        }

        void look_at(double x, double y, double z, string frame){
            control_msgs::PointHeadGoal goal;

            goal.target.header.stamp = ros::Time::now();
            goal.target.header.frame_id = frame;
            goal.target.point.x = x;
            goal.target.point.y = y;
            goal.target.point.z = z;
            goal.min_duration = ros::Duration(1.0);
            client->sendGoal(goal);
            client->waitForResult();

        }
};

void scanScene(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){

    // point cloud data
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sum_pcPtr(new pcl::PointCloud<pcl::PointXYZRGB>());
    sensor_msgs::PointCloud2 output;
    pcl::PointCloud<pcl::PointXYZRGB> cloud;

    geometry_msgs::PointStamped pt_head;
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::TransformStamped initial_transformStamped;

    // transformation
    tf::Transform initial_transform;
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    // head camera
    HeadAction* head_action = new HeadAction();

    // keep the initial transformation to transform the point cloud into the same frame
    pt_head.header.frame_id = "head_camera_rgb_optical_frame";
    pt_head = buffer.transform(pt_head, "base_link");
    ROS_INFO("the head position point: %f, %f, %f", pt_head.point.x, pt_head.point.y, pt_head.point.z);

    // look at the half point of Fetch's body
    head_action->look_at(1.0, 0.0, pt_head.point.z / 2, "base_link");
    ros::Duration(2.0).sleep();

    initial_transformStamped = buffer.lookupTransform("head_camera_rgb_optical_frame", "base_link",ros::Time(),ros::Duration(3.0));
    // cout << initial_transformStamped << endl;
    tf::transformMsgToTF(initial_transformStamped.transform, initial_transform);

    // ------------------------- CHANGE HERE -------------------------
    int degree_end = 45;
    int degree_start = -45;
    // ---------------------------------------------------------------

    int full_range = degree_end - degree_start;
    int sampling_time = full_range / 30 + 1;

    for (int i = 0; i < sampling_time; i++){
        // 여기에서 subscriber가 cur_degree 마다 맞는 msg를 전달해줘야 함. 현재는 여기서 한번만 받는

        double cur_degree = degree_start + i*30;
        ROS_INFO("current degree %f, point %f, %f, %f ", cur_degree, 1.0, sin( cur_degree * PI / 180.0 ), pt_head.point.z);
        head_action->look_at(1.0, sin( cur_degree * PI / 180.0 ), pt_head.point.z / 2, "base_link");
        ros::Duration(2.0).sleep();

        //the transformation is to transform the point cloud into the same frame
        transformStamped = buffer.lookupTransform("base_link", "head_camera_rgb_optical_frame",ros::Time());
        tf::Transform transform;
        tf::transformMsgToTF(transformStamped.transform, transform);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());

        // change the input pointcloud2 msg to pcl
        pcl::fromROSMsg(*cloud_msg, cloud);
        pcl_ros::transformPointCloud (cloud, *transformed_cloud, transform);
        ros::Duration(2.0).sleep();

        // combine the transformed point cloud together to get a wider view of the scene
        *sum_pcPtr += *transformed_cloud;

        // publish output 
        pcl_ros::transformPointCloud (*transformed_cloud, cloud, initial_transform);
        pcl::toROSMsg(cloud, output);
        output.header.frame_id = "head_camera_rgb_optical_frame";
        output.header.stamp = ros::Time::now();
        pub.publish(output);
    }

    // head_action->look_at(1.0, 0.0, pt_head.point.z / 2, "base_link");
    head_action->look_at(1.0, 0.0, pt_head.point.z/ 2, "base_link");

    ros::Duration(5.0).sleep();    // ROS_INFO("Receive data");
    pcl::fromROSMsg(*cloud_msg, cloud);
    output.header.frame_id = "head_camera_rgb_optical_frame";
    output.header.stamp = ros::Time::now();
    pub.publish(output);


    ROS_INFO("finish scanning");
    ros::Duration(1.0).sleep();
}

void cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){

    // // ROS_INFO("Receive data");
    // pcl::PointCloud<pcl::PointXYZRGB> cloud;
    // pcl::fromROSMsg(*cloud_msg, cloud);

    pub.publish(cloud_msg);
}

int main(int argc, char** argv){
    
    ros::init(argc, argv, "cube_test");
    ros::NodeHandle nh;

    while (nh.ok()) {
        // Create a ROS Subscriber to IMAGE_TOPIC with a queue_size of 1 and a callback function to scanScene
        sub = nh.subscribe("/head_camera/depth_registered/points", 1, cb);
        
        pub = nh.advertise<sensor_msgs::PointCloud2>("/test/points", 1);

        // spin
        ros::spin();
    }
    // success
    return 0;
}