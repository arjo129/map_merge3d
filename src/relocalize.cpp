#include <ros/ros.h>
#include <rosbag/bag.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <map_merge3d/scancontext.h>
#include <map_merge3d/map_merge3d.h>

SCManager manager;
rosbag::Bag bag;
RobotTrail* robot;
void onPointCloud(sensor_msgs::PointCloud2 pcloud) 
{
    
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(pcloud, cloud);
    try
    {
        auto trans = robot->getEstimatedPosition(cloud);
        static tf::TransformBroadcaster br;
        
        tf::StampedTransform transformStamped(trans, pcloud.header.stamp, "map", pcloud.header.frame_id);
        br.sendTransform(transformStamped);
        ROS_INFO("got fix");
    } 
    catch(LoopNotFoundException e)
    {
        ROS_ERROR("Unable to localize.");
    }
}

int main(int argc, char**argv) {
    ros::init(argc, argv, "LoopRecorder");
    
    ros::NodeHandle nh;
    bag.open("out.index", rosbag::bagmode::Read);

    robot = new RobotTrail(bag);


    auto subscriber = nh.subscribe("velodyne_points", 1,onPointCloud);
    ros::spin();
    return 0;
}