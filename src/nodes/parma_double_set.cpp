#include <stdio.h>
#include <string.h>
#include <stdlib.h>

//ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <kdl/frames_io.hpp>
#include "geometry_msgs/Pose.h"

//OPENCV
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

//PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>

//CUSTOM NODES
#include "lar_tools.h"
#include "commons/lar_vision_commons.h"

//boost
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>

using namespace std;

//ROS
ros::NodeHandle* nh;

typedef pcl::PointXYZ PointType;
typedef pcl::PointXYZRGB PointTypeColored;
typedef pcl::Normal NormalType;

//CLOUDS & VIEWER
pcl::visualization::PCLVisualizer* viewer;

//CLouds
pcl::PointCloud<PointType>::Ptr cloud_1(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud_trans_1(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud_trans_filtered_1(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud_2(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud_trans_2(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud_trans_filtered_2(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud_scene(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud_scene_boxed(new pcl::PointCloud<PointType>);


//filters
pcl::PassThrough<PointType> pass_filter;

//Sorter for Target Objects
struct TargetObject {
        pcl::PointCloud<PointTypeColored>::Ptr cloud;
        Eigen::Vector4f centroid;

        TargetObject(pcl::PointCloud<PointTypeColored>::Ptr& cloud_in){
                cloud = cloud_in;
                pcl::compute3DCentroid(*cloud, centroid);
        }

};
struct TargetObjectSorterX
{
        inline bool operator() (const TargetObject& obj1, const TargetObject& obj2)
        {
                return obj1.centroid[0] < obj2.centroid[0];
        }
};
std::vector<TargetObject> target_objects;


//Topics
std::string depth_topic_1;
std::string depth_topic_2;
ros::Subscriber sub_cloud_1;
ros::Subscriber sub_cloud_2;

//Parameters
struct Box {
        double z_min;
        double z_max;
        double x_min;
        double x_max;
        double y_min;
        double y_max;
};
Box scene_bounding_box;

/** TRANSFORMS */
Eigen::Matrix4d T_0_CAMERA_1;
Eigen::Matrix4d T_0_CAMERA_2;
Eigen::Matrix4d T_0_ROBOT;

int data_to_consume = 0;


/**
 * Convert geometry_msgs::Pose to KDL::Frame
 * @param pose source geometry_msgs::Pose
 * @param frame target KDL::Frame
 */
void poseToFrame(const geometry_msgs::Pose& pose, KDL::Frame& frame) {
        KDL::Rotation r = KDL::Rotation::Quaternion(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
                );
        KDL::Vector p(
                pose.position.x,
                pose.position.y,
                pose.position.z
                );
        frame = KDL::Frame(r, p);
}

/**
 * Converts KDL::Frame to geometry_msgs::Pose
 * @param frame source KDL::Frame
 * @param pose target geometry_msgs::Pose
 */
void frameToPose(KDL::Frame& frame, geometry_msgs::Pose& pose) {
        pose.position.x = frame.p.x();
        pose.position.y = frame.p.y();
        pose.position.z = frame.p.z();

        frame.M.GetQuaternion(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
                );
}

/**
 * KDL Frame to Eigen::Matrix4d
 */
void frameToEigen(KDL::Frame& frame, Eigen::Matrix4d& t){
        t(0,0) = frame.M(0,0);
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
                           void* viewer_void) {
        if (event.getKeySym() == "v" && event.keyDown()) {

        }
}



void build_clusters(){

        pcl::search::KdTree<PointType>::Ptr tree2(new pcl::search::KdTree<PointType>);
        tree2->setInputCloud(cloud_scene_boxed);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointType> ec;
        ec.setClusterTolerance(0.02);
        ec.setMinClusterSize(100);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree2);
        ec.setInputCloud(cloud_scene_boxed);
        ec.extract(cluster_indices);

        target_objects.clear();
        for (int i = 0; i < cluster_indices.size(); i++) {
                pcl::PointCloud<PointTypeColored>::Ptr cluster(new pcl::PointCloud<PointTypeColored>);
                pcl::copyPointCloud(*cloud_scene_boxed,cluster_indices[i].indices,*cluster);
                target_objects.push_back(TargetObject(cluster));
        }
        std::sort(target_objects.begin(), target_objects.end(), TargetObjectSorterX());

}

void filter_box_cloud(pcl::PointCloud<PointType>::Ptr cloud_in,
                      Box bounding_box,
                      pcl::PointCloud<PointType>::Ptr cloud_out
                      ){

        //Z
        pass_filter.setInputCloud (cloud_in);
        pass_filter.setFilterFieldName ("z");
        pass_filter.setFilterLimits (bounding_box.z_min, bounding_box.z_max);
        pass_filter.filter(*cloud_out);

        //X
        pass_filter.setInputCloud (cloud_out);
        pass_filter.setFilterFieldName ("x");
        pass_filter.setFilterLimits (bounding_box.x_min, bounding_box.x_max);
        pass_filter.filter(*cloud_out);

        //Y
        pass_filter.setInputCloud (cloud_out);
        pass_filter.setFilterFieldName ("y");
        pass_filter.setFilterLimits (bounding_box.y_min, bounding_box.y_max);
        pass_filter.filter(*cloud_out);

}


void
cloud_1_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
        //if(cloud_consuming) return;
        pcl::PCLPointCloud2 pcl_pc;
        pcl_conversions::toPCL(*input, pcl_pc);
        pcl::fromPCLPointCloud2(pcl_pc, *cloud_1);

        pcl::transformPointCloud(*cloud_1, *cloud_trans_1, T_0_CAMERA_1);

}

void
cloud_2_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
        //if(cloud_consuming) return;
        pcl::PCLPointCloud2 pcl_pc;
        pcl_conversions::toPCL(*input, pcl_pc);
        pcl::fromPCLPointCloud2(pcl_pc, *cloud_2);

        pcl::transformPointCloud(*cloud_2, *cloud_trans_2, T_0_CAMERA_2);

}

void build_scene(){

        cloud_scene = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);

        (*cloud_scene) += (*cloud_trans_1);
        (*cloud_scene) += (*cloud_trans_2);

        filter_box_cloud(cloud_scene,scene_bounding_box,cloud_scene_boxed);
}

/** MAIN NODE **/
int
main(int argc, char** argv) {

        // Initialize ROS
        ros::init(argc, argv, "parma_double_set");
        ROS_INFO("parma_double_set node started...");
        nh = new ros::NodeHandle("~");

        nh->param<std::string>("depth_topic_1", depth_topic_1, "/vrep/camera_depth_1");
        nh->param<std::string>("depth_topic_2", depth_topic_2, "/vrep/camera_depth_2");
        nh->param<double>("bounding_box_x_min", scene_bounding_box.x_min, 0.0);
        nh->param<double>("bounding_box_x_max", scene_bounding_box.x_max, 1.0);
        nh->param<double>("bounding_box_y_min", scene_bounding_box.y_min, -1.0);
        nh->param<double>("bounding_box_y_max", scene_bounding_box.y_max, 1.0);
        nh->param<double>("bounding_box_z_min", scene_bounding_box.z_min, 0.01);
        nh->param<double>("bounding_box_z_max", scene_bounding_box.z_max, 2.0);

        /** VIEWER */
        viewer = new pcl::visualization::PCLVisualizer("viewer");
        viewer->registerKeyboardCallback(keyboardEventOccurred, (void*) &viewer);

        /** TRANSFORMS */
        lar_tools::create_eigen_4x4_d(0, 0, 0, 0,0, 0, T_0_ROBOT);
        lar_tools::create_eigen_4x4_d(0.5, -0.684, 0.616, 0,0, 0, T_0_CAMERA_1);
        lar_tools::create_eigen_4x4_d(0.5, 0.959, 0.617, 0,0, 0, T_0_CAMERA_2);

        Eigen::Matrix4d rotx,rotz;
        lar_tools::rotation_matrix_4x4_d('x',M_PI/2.0+ M_PI/6.0,rotx);
        lar_tools::rotation_matrix_4x4_d('z',M_PI,rotz);
        T_0_CAMERA_2 = T_0_CAMERA_2 * rotx;
        T_0_CAMERA_1 = T_0_CAMERA_1 * rotz;
        T_0_CAMERA_1 = T_0_CAMERA_1 * rotx;

        //Topics Subscription/Advertising
        sub_cloud_1 = nh->subscribe(depth_topic_1, 1, cloud_1_cb);
        sub_cloud_2 = nh->subscribe(depth_topic_2, 1, cloud_2_cb);


        // Spin & Time
        ros::Rate r(10);
        // Spin
        while (nh->ok() && !viewer->wasStopped()) {

                build_scene();
                build_clusters();
                
                viewer->removeAllPointClouds();
                viewer->removeAllShapes();
                viewer->addPointCloud(cloud_scene_boxed,"scene");

                lar_vision::Palette palette;

                std::stringstream ss;

                for(int i = 0 ; i < target_objects.size(); +i++){
                  Eigen::Vector3i color = palette.getColor();
                  ss.str("");
                  ss << "target_object_"<<i;
                  lar_vision::display_cloud(*viewer,target_objects[i].cloud, color[0],color[1],color[2], 2.0, ss.str());

                }

                //viewer->addPointCloud(cloud_trans_2,"cloud_2");
                lar_vision::draw_reference_frame(*viewer, T_0_ROBOT, 1.0, "T_0_ROBOT");

                viewer->spinOnce();
                ros::spinOnce();
                r.sleep();
        }

}
