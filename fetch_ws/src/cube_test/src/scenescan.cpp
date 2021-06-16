#include <iostream>
#include <ros/ros.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h> 
#include <pcl_ros/transforms.h> 

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <control_msgs/PointHeadAction.h>
#include <actionlib/client/simple_action_client.h>

const double PI = 3.141592;

typedef actionlib::SimpleActionClient<control_msgs::PointHeadAction> PointHeadClient;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void head_lookat(double x, double y, double z, std::string frame_id){
        PointHeadClient* client = new PointHeadClient("head_controller/point_head", true);
        //the goal message we will be sending
        control_msgs::PointHeadGoal goal;

        //the target point, expressed in the requested frame
        goal.target.header.frame_id = frame_id;
        goal.target.point.x = x; 
        goal.target.point.y = y; 
        goal.target.point.z = z;
        goal.min_duration = ros::Duration(0.5);

        //send the goal
        client->sendGoal(goal);

        //wait for it to get there (abort after 2 secs to prevent getting stuck)
        client->waitForResult(ros::Duration(2));
};

void PerceptionClustering::scanScene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_pcPtr, int degree_start, int degree_end)
{
    // get the height of the head pose and look at the origin at half of the head height
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sum_pcPtr(new pcl::PointCloud<pcl::PointXYZRGB>());
    geometry_msgs::PointStamped pt_head;
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::TransformStamped initial_transformStamped;
    tf::Transform initial_transform;
    tf::TransformListener listener;
    
    // keep the initial transformation to transform the point cloud into the same frame
    tf::Transform(pt_head.point, tf::Vector3(0.0, 0.0, 0.0), "base_link", "head_camera_rgb_optical_frame");
    ROS_INFO("the head position point: %f, %f, %f", pt_head.point.x, pt_head.point.y, pt_head.point.z);
    head_lookat(1.0, 0.0, pt_head.point.z / 2, "base_link");
    // the time out is for the head to slide into the exact pose
    ros::Duration(4.0).sleep();

    initial_transformStamped = listener.lookupTransform( "head_camera_rgb_optical_frame", "base_link",ros::Time(0));
    tf::transformMsgToTF(initial_transformStamped.transform, initial_transform);
    int full_range = degree_end - degree_start;
    int sampling_time = full_range / 30 + 1;

    for (int i = 0; i < sampling_time; i++){    
        double cur_degree = degree_start + i*30;
        ROS_INFO("current degree %f, point %f, %f, %f ", cur_degree, 1.0, sin( cur_degree * PI / 180.0 ), pt_head.point.z);
        head_lookat(1.0, sin( cur_degree * PI / 180.0 ), pt_head.point.z / 2, "base_link");
        ros::Duration(5.0).sleep();

            //the transformation is to transform the point cloud into the same frame
        transformStamped = listener.lookupTransform("base_link", "head_camera_rgb_optical_frame");
        tf::Transform transform;
        tf::transformMsgToTF(transformStamped.transform, transform);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
        pcl_ros::transformPointCloud (*input_pcPtr_, *transformed_cloud, transform);
        ros::Duration(2.0).sleep();
        // combine the transformed point cloud together to get a wider view of the scene
        *sum_pcPtr += *transformed_cloud;

    }
    head_lookat(1.0, 0.0, pt_head.point.z / 2, "base_link");
    ros::Duration(3.0).sleep();
    pcl_ros::transformPointCloud (*sum_pcPtr, *input_pcPtr, initial_transform);

    ROS_INFO("finish scanning");
    ros::Duration(1.0).sleep();
}

int main(int argc, char** argv){
    ros::init(argc, argv, "scenescan");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<PointCloud>("/head_camera/depth_registered/points", 1);

    PerceptionClustering::scanScene*(sub,-30, 30);
    ros::spin();
}