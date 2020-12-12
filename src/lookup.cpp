#include <ros/ros.h>
#include <map_merge3d/map_merge3d.h>
#include <map_merge3d/scancontext.h>
#include <geometry_msgs/Point.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <cstdlib>

int main(int argc, char**argv) {
    
    rosbag::Bag bag, bag2;
    bag.open("/media/arjo/ba3893af-3249-49c4-8441-fb741cff5e261/test_data_sets/loop1.bag", rosbag::BagMode::Read);
    bag2.open("/media/arjo/ba3893af-3249-49c4-8441-fb741cff5e261/test_data_sets/loop2.bag", rosbag::BagMode::Read);

    RobotTrail robot(bag), robot2(bag2);

    robot.merge(robot2);

    return 0;
}