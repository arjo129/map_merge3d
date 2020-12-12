#include <ros/ros.h>
#include <rosbag/bag.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>

#include <map_merge3d/scancontext.h>

SCManager manager;
rosbag::Bag bag;
tf::TransformListener* listener;

void onPointCloud(sensor_msgs::PointCloud2 pcloud) 
{
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(pcloud, cloud);
    manager.makeAndSaveScancontextAndKeys(pcloud.header, cloud);
    auto length = manager.get_number_of_entries();

    auto desc = manager.getDescriptorId(length-1);

    tf::StampedTransform transform;
    try 
    {
        listener->waitForTransform(pcloud.header.frame_id,"map", pcloud.header.stamp, ros::Duration(1.));
        listener->lookupTransform(pcloud.header.frame_id, "map", pcloud.header.stamp, transform);

        auto origin = transform.getOrigin();
        auto rotation = transform.getRotation();
        geometry_msgs::TransformStamped pt;
        pt.header.stamp = pcloud.header.stamp;
        pt.transform.translation.x = origin.x();
        pt.transform.translation.y = origin.y();
        pt.transform.translation.z = origin.z();
        pt.transform.rotation.x = rotation.x();
        pt.transform.rotation.y = rotation.y();
        pt.transform.rotation.z = rotation.z();
        pt.transform.rotation.w = rotation.w();
        bag.write("descriptors", pcloud.header.stamp, desc);
        bag.write("position", pcloud.header.stamp, pt);

    }
    catch ( tf::LookupException e) 
    {
        ROS_ERROR("Error getting transform %s", e.what());
    }
    catch ( tf::ConnectivityException e) 
    {
        ROS_ERROR("Error getting transform %s", e.what());
    }
     catch ( tf::ExtrapolationException e) 
    {
        ROS_ERROR("Error getting transform %s", e.what());
    }
}

int main(int argc, char**argv) {
    ros::init(argc, argv, "LoopRecorder");
    ros::NodeHandle nh;
    bag.open("out.index", rosbag::bagmode::Write);

    auto subscriber = nh.subscribe("velodyne_points", 1,onPointCloud);
    listener = new tf::TransformListener();
    ros::spin();
    return 0;
}