
#include <ros/ros.h>


#include <cstdlib>

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include "lar_tools.h"
#include "lar_vision_commons.h"

//Parames
#include <dynamic_reconfigure/server.h>
#include <lar_vision/CameraErrorReductionConfig.h>



using namespace std;
using namespace lar_vision;

cv::Mat img;
std::vector<cv::Point2f> points;
cv::Point2f center;

//Ros parameters
ros::NodeHandle* nh;

double width = 640;
double height = 480;
double min_x = -1.0;
double max_x = 1.0;
double min_y = -1.0;
double max_y = 1.0;
double min_z = 0.0;
double max_z = 2.0;


std::string cloud_path = "";
std::string cloud2_path = "";

std::vector<pcl::PointCloud<PointType>::Ptr> clouds;
std::vector<Eigen::Matrix4d> robot_poses;
std::vector<Eigen::Matrix4d> ee_poses;
Eigen::Matrix4d correction;

//pcl::PointCloud<PointType>::Ptr cloud_diff(new pcl::PointCloud<PointType>);
pcl::visualization::PCLVisualizer* viewer;
bool compute_difference = true;
bool export_solid = false;



void showClouds(){

        lar_vision::Palette palette;
        viewer->removeAllPointClouds();
        for(int i = 0; i < clouds.size(); i++) {
                Eigen::Vector3i color = palette.getColor();
                std::string name =  "cloud"+boost::lexical_cast<std::string>(i);

                Eigen::Matrix4d robot_pose = robot_poses[i];
                Eigen::Matrix4d ee_pose = ee_poses[i];
                ee_pose = ee_pose*correction;
                Eigen::Matrix4d camera_pose = robot_pose*ee_pose;



                pcl::PointCloud<PointType>::Ptr cloud_trans(new pcl::PointCloud<PointType>);
                pcl::transformPointCloud(*clouds[i],*cloud_trans,camera_pose);


                pcl::PassThrough<PointType> pass;
                pass.setInputCloud (cloud_trans);
                pass.setFilterFieldName ("z");
                pass.setFilterLimits (min_z, max_z);
                //pass.setFilterLimitsNegative (true);
                pass.filter (*cloud_trans);


                lar_vision::display_cloud(*viewer, cloud_trans, color(0),color(1),color(2), 2,name);

        }

}

void callback(lar_vision::CameraErrorReductionConfig& config, uint32_t level) {
        ROS_INFO("Reconfigure Request:");

        min_z = config.min_z;
        max_z = config.max_z;  

        lar_tools::create_eigen_4x4_d(
          config.x,
          config.y,
          config.z,
          config.roll,
          config.pitch,
          config.yaw,
          correction
        );
        showClouds();
}




int main(int argc, char** argv) {

        // Initialize ROS
        lar_tools::init_ros_node(argc, argv, "camera_error_reduction");
        nh = new ros::NodeHandle("~");

        std::string path = "~/temp/temp_clouds/";
        std::string output_name = "merge_results";

        nh->param<std::string>("folder", path, "~/temp/temp_clouds/");


        lar_tools::create_eigen_4x4_d(
          0,0,0,
          0,0,0,
          correction
        );

        viewer = new pcl::visualization::PCLVisualizer("viewer");

        //Load
        for(int i =0; i < 5; i++) {
                std::string cloud_path = path +"/"+ boost::lexical_cast<std::string>(i) +".pcd";
                std::string robot_pose_path = path +"/"+ boost::lexical_cast<std::string>(i) +"_robot.txt";
                std::string ee_pose_path = path +"/"+ boost::lexical_cast<std::string>(i) +"_ee.txt";

                clouds.push_back(pcl::PointCloud<PointType>::Ptr (new pcl::PointCloud<PointType>()));
                pcl::io::loadPCDFile<PointType> (cloud_path, *clouds[i]);

                Eigen::Matrix4d robot_pose = lar_tools::load_transform_4x4_d(robot_pose_path);
                Eigen::Matrix4d ee_pose = lar_tools::load_transform_4x4_d(ee_pose_path);
                robot_poses.push_back(robot_pose);
                ee_poses.push_back(ee_pose);
        }

        showClouds();

        //Configurations
        dynamic_reconfigure::Server<lar_vision::CameraErrorReductionConfig> srv;
        dynamic_reconfigure::Server<lar_vision::CameraErrorReductionConfig>::CallbackType f;
        f = boost::bind(&callback, _1, _2);
        srv.setCallback(f);



        while (nh->ok() && !viewer->wasStopped()) {

                viewer->spinOnce();
                ros::spinOnce();
        }


        return 0;
}
