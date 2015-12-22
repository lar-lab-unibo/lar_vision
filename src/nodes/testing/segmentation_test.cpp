#include <ros/ros.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <boost/thread.hpp>
#include <boost/date_time.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include "lar_tools.h"
#include "lar_vision_commons.h"
#include "segmentation/HighMap.h"
#include "segmentation/OnePointRansac.h"
#include "grasping/Slicer.h"
#include "Grasper.h"
#include "CrabbyGripper.h"

#include "boost/date_time/posix_time/posix_time.hpp"

using namespace lar_tools;
using namespace lar_vision;
using namespace boost::posix_time;

void load_transform(std::string path, Eigen::Matrix4f& t) {
        //LOAD MATRIX
        ifstream myReadFile;
        myReadFile.open(path.c_str());
        char output[100];
        if (myReadFile.is_open()) {
                for (int i = 0; i < 4; i++) {
                        for (int j = 0; j < 4; j++) {
                                myReadFile >> output;
                                t(i, j) = atof(output);
                        }
                }
        } else {
                std::cout << t << std::endl;
        }
        myReadFile.close();
}

/** MAIN NODE **/
int
main(int argc, char** argv) {

        // Initialize ROS
        lar_tools::init_ros_node(argc, argv, "segmentation_test");
        ros::NodeHandle nh("~");

        //PCL
        pcl::visualization::PCLVisualizer* viewer = new pcl::visualization::PCLVisualizer("viewer");


        std::string cloud_path;
        std::string transform_path;
        double slice_size;
        double object_slice_size;
        double offset;
        double reduction;
        double filter_leaf;
        bool slice = false;
        bool grasp = false;
        double grasp_alpha,grasp_delta;
        double grasp_max_curvature;

        double angle_th = 10.0f;
        int min_inliers = 50;
        int test_iterations = 5;

        std::string segmentation_type;

        nh.param<std::string>("cloud", cloud_path, "");
        nh.param<std::string>("transform_path", transform_path, "");
        nh.param<double>("slice_size", slice_size, 0.01);
        nh.param<double>("object_slice_size", object_slice_size, 0.01);
        nh.param<double>("angle_th", angle_th, 20.0);
        nh.param<int>("min_inliers", min_inliers, 50);
        nh.param<int>("test_iterations", test_iterations, 5);
        nh.param<double>("offset", offset, 0.0);
        nh.param<double>("reduction", reduction, 1.01);
        nh.param<double>("filter_leaf", filter_leaf, 0.0051);
        nh.param<bool>("doslice", slice, false);
        nh.param<bool>("dograsp", grasp, false);
        nh.param<double>("grasp_alpha", grasp_alpha, 0.1f);
        nh.param<double>("grasp_delta", grasp_delta, 0.005f);
        nh.param<double>("grasp_max_curvature", grasp_max_curvature, 1.0);
        nh.param<std::string>("segmentation_type", segmentation_type, "heightmap");



        ROS_INFO("Cloud Path:  %s", cloud_path.c_str());


        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr cloud_filtered(new pcl::PointCloud<PointType>());
        pcl::PointCloud<NormalType>::Ptr cloud_normals(new pcl::PointCloud<NormalType>());
        Eigen::Matrix4f base_transform;

        //Load
        pcl::io::loadPCDFile<PointType> (cloud_path, *cloud);
        load_transform(transform_path, base_transform);
        if (transform_path.compare("") != 0) {
                pcl::transformPointCloud(*cloud, *cloud, base_transform);
        } else {
                lar_tools::create_eigen_4x4(0, 0, 0, 0, 0, 0, base_transform);
        }

        // Create the filtering object
        pcl::VoxelGrid<PointType> sor;
        sor.setInputCloud(cloud);
        sor.setLeafSize(filter_leaf, filter_leaf, filter_leaf);
        sor.filter(*cloud_filtered);



        //Normals
        compute_normals(cloud_filtered, cloud_normals);

        pcl::PointCloud<PointType>::Ptr planes(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr clusters(new pcl::PointCloud<PointType>());
        std::vector<int> filtered_indices;
        std::vector<int> planes_indices;

        //Segmentation
        for(int i = 0; i < test_iterations; i++) {
                filtered_indices.clear();
                planes_indices.clear();

                ptime time_start,time_end;
                time_duration duration;

                if(segmentation_type=="heightmap") {
                        HighMap map(3.0f, slice_size, offset, reduction);

                        time_start =microsec_clock::local_time();

                        map.planesCheck(
                                cloud_filtered,
                                cloud_normals,
                                filtered_indices,
                                planes_indices,
                                angle_th,
                                min_inliers
                                );
                        //... execution goes here ...
                        time_end =microsec_clock::local_time();
                        duration = time_end - time_start;


                }else if(segmentation_type=="onepoint") {
                        OnePointRansac one(0.01,0.99,1000,std::cos(angle_th*M_PI/180.0));

                        time_start =microsec_clock::local_time();

                        one.planesCheck(
                                cloud_filtered,
                                cloud_normals,
                                filtered_indices,
                                planes_indices,
                                angle_th,
                                min_inliers
                                );

                        time_end =microsec_clock::local_time();
                        duration = time_end - time_start;


                }else{
                        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
                        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
                        // Create the segmentation object
                        pcl::SACSegmentation<PointType> seg;
                        seg.setOptimizeCoefficients (true);
                        seg.setModelType (pcl::SACMODEL_PLANE);
                        seg.setMethodType (pcl::SAC_RANSAC);
                        seg.setDistanceThreshold (0.01);
                        seg.setInputCloud (cloud_filtered);

                        time_start =microsec_clock::local_time();

                        seg.segment (*inliers, *coefficients);

                        pcl::ExtractIndices<PointType> extract;
                        extract.setInputCloud (cloud_filtered);
                        extract.setIndices (inliers);

                        extract.setNegative (false);
                        extract.filter (planes_indices);

                        extract.setNegative (true);
                        extract.filter (filtered_indices);

                        time_end =microsec_clock::local_time();
                        duration = time_end - time_start;

                }

                std::cout << "Type: "<<segmentation_type<<" size: "<<cloud_filtered->points.size()<< " time: "<< duration.total_milliseconds()<<std::endl;
                (*cloud_filtered)+=(*cloud_filtered);
                (*cloud_normals)+=(*cloud_normals);
        }

        if(test_iterations==1) {
                pcl::copyPointCloud(*cloud_filtered, filtered_indices, *clusters);
                pcl::copyPointCloud(*cloud_filtered, planes_indices, *planes);

                display_cloud(*viewer, clusters, 255,255,255, 1, "clusters");
                display_cloud(*viewer, planes, 255,0,0, 1, "planes");
        }

        while (nh.ok() && !viewer->wasStopped()) {
                viewer->spinOnce();
                ros::spinOnce();
        }
}
