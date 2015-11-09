#include <ros/ros.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <boost/thread.hpp>
#include <boost/date_time.hpp>

#include <pcl/visualization/pcl_visualizer.h>

#include "lar_tools.h"
#include "lar_vision_commons.h"
#include "segmentation/HighMap.h"

using namespace lar_tools;
using namespace lar_vision;

/** MAIN NODE **/
int
main(int argc, char** argv) {

    // Initialize ROS
    lar_tools::init_ros_node(argc, argv, "tesing_node");
    ros::NodeHandle nh("~");

    //PCL
    pcl::visualization::PCLVisualizer* viewer = new pcl::visualization::PCLVisualizer("viewer");


    std::string cloud_path;
    double slice_size;
    double offset;
    double reduction;

    nh.param<std::string>("cloud", cloud_path, "");
    nh.param<double>("slice_size", slice_size, 0.01f);
    nh.param<double>("offset", offset, 0.0f);
    nh.param<double>("reduction", reduction, 1.0f);
    

    ROS_INFO("Cloud Path:  %s", cloud_path.c_str());


    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::io::loadPCDFile<pcl::PointXYZRGBA> (cloud_path, *cloud);


    HighMap map(2.0f, slice_size, offset, reduction);

    viewer->addPointCloud(cloud, "cloud");
    while (nh.ok() && !viewer->wasStopped()) {
        viewer->spinOnce();
        ros::spinOnce();
    }
}
