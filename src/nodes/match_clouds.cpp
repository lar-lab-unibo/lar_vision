
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

#include "lar_tools.h"
#include "lar_vision_commons.h"

//Parames
#include <dynamic_reconfigure/server.h>
#include <lar_vision/MatchCloudsConfig.h>



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
pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud2(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud_diff(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud_boxed(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud2_boxed(new pcl::PointCloud<PointType>);
pcl::visualization::PCLVisualizer* viewer;
bool compute_difference = true;

void boxCloud(pcl::PointCloud<PointType>::Ptr& cloud,pcl::PointCloud<PointType>::Ptr& cloud_boxed){

        pcl::PassThrough<PointType> pass;
        pass.setInputCloud (cloud);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (min_x,max_x);
        pass.filter (*cloud_boxed);

        pass.setInputCloud (cloud_boxed);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (min_y,max_y);
        pass.filter (*cloud_boxed);

        pass.setInputCloud (cloud_boxed);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (min_z,max_z);
        pass.filter (*cloud_boxed);
}


void showClouds(){


        if(!compute_difference) {
                viewer->removeAllPointClouds();
                display_cloud(*viewer, cloud_boxed, 0,255,0, 2, "cloud");
                display_cloud(*viewer, cloud2_boxed, 255,0,0, 2, "cloud2");
        }else{
                pcl::search::KdTree<PointType> tree;
                tree.setInputCloud(cloud2_boxed);

                std::vector<int> found_indices;
                std::vector<float> found_distances;

                float avg_min_square = 0.0f;
                float counter = 0;
                float max_distance = 0.0f;
                int max_index = -1;
                int max_cad_index = -1;
                cloud_diff->points.clear();
                for (int i = 0; i < cloud_boxed->points.size(); i++) {
                        std::cout << (double)i/(double)cloud_boxed->points.size()<<"%\n";
                        PointType search_p;
                        search_p.x = cloud_boxed->points[i].x;
                        search_p.y = cloud_boxed->points[i].y;
                        search_p.z = cloud_boxed->points[i].z;
                        found_indices.clear();
                        found_distances.clear();
                        int n = tree.radiusSearch(search_p, 0.05f, found_indices, found_distances);

                        int color  = 0;
                        int color2  = 0;
                        if (n > 0) {
                                Eigen::Vector3f p1, p2;
                                p1 << cloud_boxed->points[i].x, cloud_boxed->points[i].y, cloud_boxed->points[i].z;
                                p2 << cloud2_boxed->points[found_indices[0]].x, cloud2_boxed->points[found_indices[0]].y, cloud2_boxed->points[found_indices[0]].z;
                                float norm = (p1-p2).norm();
                                avg_min_square += norm*norm; // found_distances[0]; // * found_distances[iter_min_index];
                                color = 255*((norm)/0.01f);
                                color2 = 255*(1.0f-(norm)/0.01f);
                                counter += 1.0f;
                                search_p.r = color;
                                search_p.g = color2;
                                search_p.b = 0;
                                cloud_diff->points.push_back(search_p);
                        }

                }
                viewer->removeAllPointClouds();
                //display_cloud(*viewer, cloud2_boxed, 255,0,0, 2, "cloud2");
                viewer->addPointCloud(cloud_diff,"cloud_diff");
                std::cout<< "Mean square error: "<<avg_min_square<<std::endl;
        }
}

void callback(lar_vision::MatchCloudsConfig& config, uint32_t level) {
        ROS_INFO("Reconfigure Request:");

        /*float axial_coefficient;
           float axial_bias;
           float axial_offset;
           float lateral_coefficient;
           float lateral_default_focal;*/

        min_x = config.min_x;
        max_x = config.max_x;

        min_y = config.min_y;
        max_y = config.max_y;

        min_z = config.min_z;
        max_z = config.max_z;

        boxCloud(cloud,cloud_boxed);
        boxCloud(cloud2,cloud2_boxed);
        showClouds();
}




int main(int argc, char** argv) {

        // Initialize ROS
        lar_tools::init_ros_node(argc, argv, "match_clouds");
        nh = new ros::NodeHandle("~");

        nh->param<std::string>("cloud", cloud_path, "");
        nh->param<std::string>("cloud_2", cloud2_path, "");


        viewer = new pcl::visualization::PCLVisualizer("viewer");

        //Load
        pcl::io::loadPCDFile<PointType> (cloud_path, *cloud);
        pcl::io::loadPCDFile<PointType> (cloud2_path, *cloud2);

        //Configurations
        dynamic_reconfigure::Server<lar_vision::MatchCloudsConfig> srv;
        dynamic_reconfigure::Server<lar_vision::MatchCloudsConfig>::CallbackType f;
        f = boost::bind(&callback, _1, _2);
        srv.setCallback(f);



        while (nh->ok() && !viewer->wasStopped()) {





                viewer->spinOnce();
                ros::spinOnce();
        }

        if(compute_difference)
          pcl::io::savePCDFileBinary("/home/daniele/temp/cloud_diff.pcd", *cloud_diff);

        return 0;
}
