#pragma once

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_msgs/TFMessage.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/visualization/cloud_viewer.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
//#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>

#include "scancontext.h"
#include "laser_operations.h"


struct Trajectory {
   std::vector<ros::Time> timestamp;
   std::vector<pcl::PointCloud<pcl::PointXYZ>> cloud;
   std::vector<tf::Transform> estimatedPosition;
};



class HistoricalTrack {
public:
    std::vector<ros::Time>timeBuffer;
    std::vector< geometry_msgs::Vector3> positionBuffer;
    std::vector<geometry_msgs::Quaternion> orientationBuffer;

    void addPoint(ros::Time time, geometry_msgs::Vector3 pt, geometry_msgs::Quaternion qt) {
        timeBuffer.push_back(time);
        positionBuffer.push_back(pt);
        orientationBuffer.push_back(qt);
    }

    geometry_msgs::Vector3 getPositionAtTime(ros::Time time) {
        auto idx = std::lower_bound (timeBuffer.begin(),  timeBuffer.end(), time);
        auto dist = idx - timeBuffer.begin();
        return positionBuffer[dist];
    }

    tf::Transform getTransformAtTime(ros::Time time) {
        auto idx = std::lower_bound (timeBuffer.begin(),  timeBuffer.end(), time);
        auto dist = idx - timeBuffer.begin();
        auto pos = positionBuffer[dist];
        tf::Vector3 v(pos.x, pos.y, pos.z);
        auto rotation  = orientationBuffer[dist];
        tf::Quaternion q(rotation.x, rotation.y, rotation.z, rotation.w); 
        tf::Transform t;
        t.setOrigin(v);
        t.setRotation(q);
        return t;
    }
};


class RobotTrail {
private:
    SCManager manager;
    HistoricalTrack track;
    std::vector<pcl::PointCloud<pcl::PointXYZI> > scans;
public:
    RobotTrail(rosbag::Bag& bag) {
        std::cout << "Reading from bag...";
        std::vector<std::string> topics;
        //topics.push_back(std::string("/X1/points"));
        //topics.push_back(std::string("/tf_static"));
        //topics.push_back(std::string("/tf"));
        //topics.push_back(std::string("position"));
        rosbag::View view(bag);
        int num_seen = 0;
        for(rosbag::MessageInstance const m: view)
        {
            auto time = m.getTime();
            if(m.getDataType() == "sensor_msgs/PointCloud2" ){
                
                if(num_seen % 40 == 0) {
                    auto msg = *m.instantiate<sensor_msgs::PointCloud2>();
                    pcl::PCLPointCloud2 pcl_pc2;
                    pcl::PointCloud<pcl::PointXYZ> cl;
                    pcl::PointCloud<SCPointType> cloud;
                    pcl_conversions::toPCL(msg, pcl_pc2);
                    pcl::fromPCLPointCloud2(pcl_pc2, cl);
                    addFakeIntensity(cl, cloud);
                    if(cloud.size() <  1000) continue;
                    manager.makeAndSaveScancontextAndKeys(msg.header, cloud);
                    addPointCloud(cloud);
                }
                num_seen++;
            }
            if(m.getDataType() == "map_merge3d/ScanContextDescriptor" ){
                auto msg = *m.instantiate<map_merge3d::ScanContextDescriptor>();
                manager.addLoopClosureDescriptor(msg);
            }

            if(m.getDataType() == "tf2_msgs/TFMessage") {
                auto msg = *m.instantiate<tf2_msgs::TFMessage>();
                for(auto t: msg.transforms){
                    if(!(t.header.frame_id == "map" && t.child_frame_id == "base_link")) continue;
                    geometry_msgs::Vector3 pt;
                    geometry_msgs::Quaternion qt;
                    pt = t.transform.translation;
                    std::cout << pt << std::endl;
                    qt = t.transform.rotation;
                    track.addPoint(t.header.stamp, pt,qt);
                }
            }
             if(m.getDataType() == "geometry_msgs/TransformStamped") {
                auto msg = *m.instantiate<geometry_msgs::TransformStamped>();
                geometry_msgs::Vector3 pt;
                geometry_msgs::Quaternion qt;
                pt = msg.transform.translation;
                qt = msg.transform.rotation;
                track.addPoint(msg.header.stamp, pt,qt);
            }

        }
        std::cout << "[Done]" <<std::endl;
    }

    tf::Transform getEstimatedPosition(pcl::PointCloud<pcl::PointXYZI>& in) {
        auto descriptor = manager.getScanContext(in);
        auto id = manager.detectLoopClosureID(descriptor);
        auto time = manager.getTimeOfId(id.first);
        auto trans = track.getTransformAtTime(time);
        tf::Quaternion q;
        q.setRPY(0,0, id.second);
        auto qt = q*trans.getRotation();
        trans.setRotation(qt);
        return trans;
    }

    void addFakeIntensity(pcl::PointCloud<pcl::PointXYZ>& in, pcl::PointCloud<pcl::PointXYZI>& other){
        for(auto pt: in) {
            if(!std::isfinite(pt.x) || !std::isfinite(pt.y) ||!std::isfinite(pt.z)) continue;

            pcl::PointXYZI intensity;
            intensity.x = pt.x;
            intensity.y = pt.y;
            intensity.z = pt.z; 
            intensity.intensity = 100.0f;
            other.push_back(intensity);
        }
    }

    void addPointCloud(pcl::PointCloud<pcl::PointXYZI>& cloud) {
        pcl::PointCloud<pcl::PointXYZI> cleaned;
        for(auto pt: cloud) {
            if(!std::isfinite(pt.x) || !std::isfinite(pt.y) ||!std::isfinite(pt.z)) continue;
            cleaned.push_back(pt);
        }
        scans.push_back(cleaned);
    }

    void getLoamPC(pcl::PointCloud<pcl::PointXYZI>& cloud) {
        for(int i = 0 ; i < scans.size(); i++) {
            pcl::PointCloud<pcl::PointXYZI> transformed_cloud;
            auto time = manager.getTimeOfId(i);
            auto transform = track.getTransformAtTime(time);
            Eigen::Affine3d e;
            tf::transformTFToEigen(transform, e);
            pcl::transformPointCloud(scans[i], transformed_cloud, e);
            cloud += transformed_cloud;
        }
    }

    void getTransformedPCatId(int id, pcl::PointCloud<pcl::PointXYZI>& cloud) {
        auto time = manager.getTimeOfId(id);
        auto transform = track.getTransformAtTime(time);
        Eigen::Affine3d e;
        tf::transformTFToEigen(transform, e);
        pcl::transformPointCloud(scans[id], cloud, e);
    }

    void stripIntensity(pcl::PointCloud<pcl::PointXYZI>& cloud, pcl::PointCloud<pcl::PointXYZ>& out) {
        for(auto pt: cloud) {
            pcl::PointXYZ p_out;
            p_out.x = pt.x;
            p_out.y = pt.y;
            p_out.z = pt.z;
            out.push_back(p_out);
        }

    }

    gtsam::Pose3 tfToPose3(tf::Transform transform) {
        auto position = transform.getOrigin();
        double roll, pitch, yaw;
        auto rotation = transform.getRotation();
        tf::Matrix3x3(rotation).getRPY(roll, pitch, yaw);
        gtsam::Rot3 rot = gtsam::Rot3::Quaternion(rotation.w(), rotation.x(), rotation.y(), rotation.z());
        gtsam::Point3 point(position.x(), position.y(), position.z());
        gtsam::Pose3 pose = gtsam::Pose3::Create(rot, point);
        //gtsam::Pose2 pose(position.x(), position.y(), yaw);
        return pose;
    }

    

    void buildInitialFactorGraph(gtsam::NonlinearFactorGraph& graph, 
        std::vector<gtsam::Symbol>& symbols, gtsam::Values& val){
        Eigen::Vector3d rotNoise(0.0001,0.0001,0.01);
        bool first = true;
        tf::Transform prev_pos;
        for(int i = 0 ; i < scans.size(); i++) {
            auto time = manager.getTimeOfId(i);
            auto transform = track.getTransformAtTime(time);
            gtsam::Symbol graph_node('x', i);
            symbols.push_back(graph_node);

            if(first) {
                first = false;
                prev_pos = transform;
                tf::Transform prior_tf;    
                gtsam::Pose3 prior = tfToPose3(prior_tf);  // prior mean is at origin
                auto priorNoise = gtsam::noiseModel::Diagonal::Sigmas(
                         (gtsam::Vector(6) << rotNoise, gtsam::Vector3::Constant(0.001)).finished());
                graph.addPrior(graph_node, prior, priorNoise);
                val.insert(graph_node, tfToPose3(transform)); 
                continue;
            }

            auto odometry_tf = (prev_pos.inverse()*transform);
            auto odometry = tfToPose3(odometry_tf);
            auto prev_graph_node = symbols[symbols.size() - 2];
            
            auto odometryNoise =  gtsam::noiseModel::Diagonal::Sigmas(
                         (gtsam::Vector(6) << rotNoise, gtsam::Vector3::Constant(0.001)).finished());
            graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(prev_graph_node, graph_node, odometry, odometryNoise);
            prev_pos = transform;
            val.insert(graph_node, tfToPose3(transform)); 
        }
    }

    void buildFactorGraphBetweenIds(gtsam::NonlinearFactorGraph& graph,
        std::vector<gtsam::Symbol>& symbols, int id1, int id2) {

        bool first = true;
        tf::Transform prev_pos;
        for(int i = id1; i < id2; i++) {
            auto time = manager.getTimeOfId(i);
            auto transform = track.getTransformAtTime(time);
            gtsam::Symbol graph_node('r', i);
            symbols.push_back(graph_node);

            if(first) {
                first = false;
                prev_pos = transform;
                continue;
            }

            
            auto odometry_tf = prev_pos.inverse()*transform;
            auto odometry = tfToPose3(odometry_tf);
            auto prev_graph_node = symbols[symbols.size() - 2];
             gtsam::Vector3 rotNoise(0.0001,0.0001,0.01);
            auto odometryNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << rotNoise, gtsam::Vector3::Constant(0.001)).finished());
            prev_pos = transform;
            graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(prev_graph_node, graph_node, odometry, odometryNoise);
        }
    }

    void addMergeConstraint(gtsam::NonlinearFactorGraph& graph, gtsam::Symbol symbol1, gtsam::Symbol symbol2, gtsam::Pose3 pose) {
        auto loopClosureNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3::Constant(0.1), gtsam::Vector3::Constant(0.001)).finished());
        auto huber = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.345), loopClosureNoise);
        graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(symbol2, symbol1, pose, huber);
    }

    tf::Transform applyYaw(float yaw) {
        tf::Quaternion q;
        q.setRPY(0,0, yaw);
        tf::Vector3 v(0,0,0);
        tf::Transform t(q,v);
        return t;
    }

    void merge(RobotTrail& other) {

        auto num_entries = other.manager.get_number_of_entries();
        std::vector<std::tuple<int, int, float> > time_of_match;

        
        for(int other_id = 0; other_id < num_entries; other_id++) {
            auto other_desc = other.manager.getDescriptorId(other_id);
            try {
                auto loop = this->manager.detectLoopClosureID(other_desc);
                auto my_desc_id = loop.first;
                
                auto d2 = this->manager.getDescriptorId(loop.first);
                time_of_match.push_back(std::make_tuple(my_desc_id, other_id, loop.second));
                
            } catch (LoopNotFoundException exception) {
                //ROS_ERROR("Loop not found");
            }
        }
 
        
        gtsam::NonlinearFactorGraph factor_graph;
        std::vector<gtsam::Symbol> my_symbols;
        gtsam::Values estimate;
        this->buildInitialFactorGraph(factor_graph, my_symbols, estimate);

        std::vector<gtsam::Symbol> other_symbols;
        other.buildFactorGraphBetweenIds(factor_graph, other_symbols, 0, other.scans.size());

        bool first = true;
        tf::Transform initial_merge_estimate;
        std::set<int> my_matches, other_matches;
        //ROS_INFO("Found %d matches", time_of_match.size());
        for(int i = 39; i < time_of_match.size(); i+=20) {
          
            pcl::PointCloud<pcl::PointXYZI> other_pointcloud, my_point_cloud, final_cloud;
            //this->getLoamPC(my_point_cloud);
            //other.getLoamPC(other_pointcloud);
            auto my_match_id= std::get<0>(time_of_match[i]);
            my_matches.insert(my_match_id);
            auto other_match_id = std::get<1>(time_of_match[i]);
             other_matches.insert(other_match_id);

            //ROS_INFO("Analysing matches via ICP %d/%d", i, time_of_match.size());
            //this->getTransformedPCatId(my_match_id, my_point_cloud);
            //other.getTransformedPCatId(other_match_id, other_pointcloud);
            
            auto my_match_time = this->manager.getTimeOfId(my_match_id);
            auto other_match_time = other.manager.getTimeOfId(other_match_id);

            auto origin1_to_robot_my_match = this->track.getTransformAtTime(my_match_time);
            auto origin2_to_robot_other = other.track.getTransformAtTime(other_match_time); 

            Eigen::Affine3d origin2_to_robot_other_eig;
            tf::Quaternion q;
            q.setRPY(0,0, std::get<2>(time_of_match[i]));
            tf::Vector3 v(0,0,0);
            tf::Transform t(q,v);
            auto initial_rotation = t;
            tf::transformTFToEigen(/*origin2_to_robot_other*/t, origin2_to_robot_other_eig);
            pcl::PointCloud<pcl::PointXYZI>  other_transformed;
            pcl::transformPointCloud(other.scans[other_match_id], other_transformed, origin2_to_robot_other_eig.matrix());

            pcl::PointCloud<pcl::PointXYZ> other_stripped, other_corners, my_stripped, my_corners;
            stripIntensity(other_transformed, other_stripped);
            stripIntensity(this->scans[my_match_id], my_stripped);
            extractCorners(other_stripped, other_corners);
            extractCorners(my_stripped, my_corners);
            
            
            auto res = ICPMatchPointToPoint(my_corners, other_corners);
            auto offset = res.first;
            /*if(res.second > 0.2) {
                ROS_INFO("Rejecting point ICP did not converge properly");
                continue;
            }*/

            Eigen::Affine3d rr;
            rr.matrix() = offset.cast<double>();
            tf::transformEigenToTF(rr, t);
            if(first) {
                initial_merge_estimate = origin1_to_robot_my_match*t*initial_rotation*origin2_to_robot_other.inverse();
                first = true;
            }

            gtsam::Symbol other_match_sym('r', other_match_id), my_sym('x', my_match_id);
            this->addMergeConstraint(factor_graph, other_match_sym, my_sym, tfToPose3(t*initial_rotation));
        }
           
        

        for(int id =0; id < other.scans.size(); id++) {
            gtsam::Symbol other_sym('r', id);
            auto other_time = other.manager.getTimeOfId(id);
            auto other_pose_tf = other.track.getTransformAtTime(other_time);
            auto other_pose = tfToPose3(initial_merge_estimate*other_pose_tf);

            estimate.insert(other_sym, other_pose);
        }
 
        //factor_graph.print("Graph\n");


        gtsam::LevenbergMarquardtOptimizer optimizer(factor_graph, estimate);
        ROS_INFO("Performing optimization");
        //estimate.print("Priors:\n");
        auto result = optimizer.optimize();
        result.print("Final Result:\n");

        pcl::PointCloud<pcl::PointXYZRGB> final_pointcloud; 
        int num_nodes =0;
        for(auto point: result) {

            auto key = point.key;
            gtsam::Symbol sym(key);
            auto source = sym.chr();
            auto id = sym.index();
            
            auto pose3d = point.value.cast<gtsam::Pose3>();

            num_nodes++;
            pcl::PointCloud<pcl::PointXYZRGB> local_frame; 
            if(source == 'r') {
                 auto scan = other.scans[id];
                //cout << "r" << id <<endl;
                pcl::PointCloud<pcl::PointXYZRGB> colored_scan;
                colorizePointCloud(scan, colored_scan, 100, 0, 0);
                pcl::transformPointCloud(colored_scan, local_frame, pose3d.matrix());
            }
            else {

                auto scan = scans[id];
                //cout << "x" << id <<endl;
                pcl::PointCloud<pcl::PointXYZRGB> colored_scan;
                colorizePointCloud(scan, colored_scan, 0, 100, 0);
                pcl::transformPointCloud(colored_scan, local_frame, pose3d.matrix());
                
           }

           if(num_nodes % 3 == 0) final_pointcloud += local_frame;
            
        }
        std::ofstream os("Pose2SLAMExample.dot");
        factor_graph.saveGraph(os, result);

    
        pcl::visualization::CloudViewer viewer_robot1("MergedPointCloud");
        


        auto cloud_ptr = final_pointcloud.makeShared();
        viewer_robot1.showCloud(cloud_ptr, "cloud1");
        
        while (!viewer_robot1.wasStopped ())
        {
        }
    }

    void colorizePointCloud(pcl::PointCloud<pcl::PointXYZI> pc, pcl::PointCloud<pcl::PointXYZRGB>& out, int r, int g, int b) {
        for(auto pt: pc) {
            pcl::PointXYZRGB new_pt;
            new_pt.r = r;
            new_pt.g = g;
            new_pt.b = b;
            new_pt.x = pt.x;
            new_pt.y = pt.y;
            new_pt.z = pt.z;

            out.push_back(new_pt);
        }
    }
};