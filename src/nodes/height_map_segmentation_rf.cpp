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
#include <pcl-1.8/pcl/common/transforms.h>

#include "lar_tools.h"
#include "lar_vision_commons.h"
#include "segmentation/HighMap.h"
#include "grasping/Slicer.h"

using namespace lar_tools;
using namespace lar_vision;

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
    lar_tools::init_ros_node(argc, argv, "tesing_node");
    ros::NodeHandle nh("~");

    //PCL
    pcl::visualization::PCLVisualizer* viewer = new pcl::visualization::PCLVisualizer("viewer");


    std::string cloud_path;
    std::string transform_path;
    double slice_size;
    double offset;
    double reduction;
    double filter_leaf;
    bool slice = false;

    nh.param<std::string>("cloud", cloud_path, "");
    nh.param<std::string>("transform_path", transform_path, "");
    nh.param<double>("slice_size", slice_size, 0.01);
    nh.param<double>("offset", offset, 0.0);
    nh.param<double>("reduction", reduction, 1.01);
    nh.param<double>("filter_leaf", filter_leaf, 0.0051);
    nh.param<bool>("doslice", slice, false);


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


    //Segmentation
    pcl::PointCloud<PointType>::Ptr planes(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr clusters(new pcl::PointCloud<PointType>());
    std::vector<int> filtered_indices;
    std::vector<int> planes_indices;

    HighMap map(2.0f, slice_size, offset, reduction);
    map.planesCheck(
            cloud_filtered,
            cloud_normals,
            filtered_indices,
            planes_indices,
            10.0f,
            500
            );
    pcl::copyPointCloud(*cloud_filtered, filtered_indices, *clusters);
    pcl::copyPointCloud(*cloud_filtered, planes_indices, *planes);


    //Clusterization
    Slicer slicer(slice_size);
    Palette palette;
    std::vector<pcl::PointIndices> cluster_indices;
    clusterize(clusters, cluster_indices);
    Eigen::Vector3f gravity;
    gravity << 0, 0, 1;
    for (int i = 0; i < cluster_indices.size(); i++) {
        pcl::PointCloud<PointType>::Ptr cluster(new pcl::PointCloud<PointType>());
        pcl::copyPointCloud(*clusters, cluster_indices[i].indices, *cluster);

        std::string name = "cluster_" + boost::lexical_cast<std::string>(i);
        Eigen::Vector3i color = palette.getColor();
        if (!slice)
            display_cloud(*viewer, cluster, color[0], color[1], color[2], 1, name);

        name = "cluster_rf_" + boost::lexical_cast<std::string>(i);
        Eigen::Vector4f centroid;
        Eigen::Vector3f center;
        pcl::ReferenceFrame rf;
        pcl::ReferenceFrame rf_oriented;
        pcl::compute3DCentroid(*cluster, centroid);
        center << centroid[0], centroid[1], centroid[2];
        compute_centroid_local_rf(cluster, rf);
        compute_centroid_local_rf(cluster, rf_oriented, gravity);
        draw_reference_frame(*viewer, center, rf_oriented, 0.1f, name);

        if (slice) {
            Eigen::Matrix4f local_transform;
            convert_rf_to_eigen_4x4(rf_oriented,local_transform);
            local_transform = lar_tools::invert_transform_4x4(local_transform);
            
            pcl::PointCloud<PointType>::Ptr cluster_oriented(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*cluster,*cluster_oriented,local_transform);
            std::vector<pcl::PointIndices> slice_indices;
            slicer.slice(cluster_oriented, slice_indices);
            std::cout << "Sliced: "<<slice_indices.size()<<std::endl;
            for (int j = 0; j < slice_indices.size(); j++) {
                pcl::PointCloud<PointType>::Ptr slice(new pcl::PointCloud<PointType>());
                pcl::copyPointCloud(*cluster, slice_indices[j].indices, *slice);
                name = "cluster_" + boost::lexical_cast<std::string>(i) + "_slice_" + boost::lexical_cast<std::string>(j);
                color = palette.getColor();
                display_cloud(*viewer, slice, color[0], color[1], color[2], 2, name);
            }

        }
    }

    //Draw

    display_cloud(*viewer, planes, 255, 0, 0, 1, "planes");

    pcl::ReferenceFrame rf;
    rf.x_axis[0] = 1.0f;
    rf.x_axis[1] = 0.0f;
    rf.x_axis[2] = 0.0f;

    rf.y_axis[0] = 0.0f;
    rf.y_axis[1] = 1.0f;
    rf.y_axis[2] = 0.0f;

    rf.z_axis[0] = 0.0f;
    rf.z_axis[1] = 0.0f;
    rf.z_axis[2] = 1.0f;

    Eigen::Vector3f center;
    center << 0, 0, 0;

    draw_reference_frame(*viewer, center, rf, 1.0f, "base");


    while (nh.ok() && !viewer->wasStopped()) {
        viewer->spinOnce();
        ros::spinOnce();
    }
}



