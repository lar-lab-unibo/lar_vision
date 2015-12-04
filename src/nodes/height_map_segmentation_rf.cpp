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
#include "Grasper.h"
#include "CrabbyGripper.h"

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

        nh.param<std::string>("cloud", cloud_path, "");
        nh.param<std::string>("transform_path", transform_path, "");
        nh.param<double>("slice_size", slice_size, 0.01);
        nh.param<double>("object_slice_size", object_slice_size, 0.01);
        nh.param<double>("angle_th", angle_th, 20.0);
        nh.param<int>("min_inliers", min_inliers, 50);
        nh.param<double>("offset", offset, 0.0);
        nh.param<double>("reduction", reduction, 1.01);
        nh.param<double>("filter_leaf", filter_leaf, 0.0051);
        nh.param<bool>("doslice", slice, false);
        nh.param<bool>("dograsp", grasp, false);
        nh.param<double>("grasp_alpha", grasp_alpha, 0.1f);
        nh.param<double>("grasp_delta", grasp_delta, 0.005f);
        nh.param<double>("grasp_max_curvature", grasp_max_curvature, 1.0);



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
                angle_th,
                min_inliers
                );
        pcl::copyPointCloud(*cloud_filtered, filtered_indices, *clusters);
        pcl::copyPointCloud(*cloud_filtered, planes_indices, *planes);


        //Clusterization
        Slicer slicer(object_slice_size);
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
                if (grasp) {
                        Eigen::Matrix4f fake_rf;
                        rotation_matrix_4x4('x', 0, fake_rf);
                        convert_rf_to_eigen_4x4(rf_oriented, fake_rf, true);
                }
                draw_reference_frame(*viewer, center, rf_oriented, 0.1f, name);
                draw_text_3D(*viewer,name,center,255,255,255,0.01f,name+"text");

                if (slice) {
                        pcl::PointCloud<PointType>::Ptr cluster_oriented(new pcl::PointCloud<PointType>());
                        Eigen::Matrix4f local_transform;
                        convert_rf_to_eigen_4x4(rf_oriented, local_transform);
                        local_transform = lar_tools::invert_transform_4x4(local_transform);
                        pcl::transformPointCloud(*cluster, *cluster_oriented, local_transform);

                        std::vector<pcl::PointIndices> slice_indices;
                        slicer.slice(cluster_oriented, slice_indices);

                        for (int j = 0; j < slice_indices.size(); j++) {
                                pcl::PointCloud<PointType>::Ptr slice(new pcl::PointCloud<PointType>());
                                pcl::PointCloud<PointType>::Ptr slice_oriented(new pcl::PointCloud<PointType>());

                                pcl::copyPointCloud(*cluster, slice_indices[j].indices, *slice);
                                pcl::copyPointCloud(*cluster_oriented, slice_indices[j].indices, *slice_oriented);

                                name = "cluster_" + boost::lexical_cast<std::string>(i) + "_slice_" + boost::lexical_cast<std::string>(j);
                                color = palette.getColor();
                                display_cloud(*viewer, slice, color[0], color[1], color[2], 2, name);
                                //                display_cloud(*viewer, slice_oriented, color[0], color[1], color[2], 2, name + "sor");

                                if (grasp && slice_indices.size()>3) {
                                        if (j == slice_indices.size()/2) {
                                                ROS_INFO("Grasping point search: %f, %f",grasp_alpha, grasp_delta);
                                                Grasper grasper(grasp_alpha, grasp_delta);
                                                grasper.setCloud(slice_oriented);
                                                display_cloud(*viewer, grasper.hull, 255, 255, 255, 10, name + "_hull");

                                                CrabbyGripper gripper;
                                                gripper.auto_discard_planar_invalids = true;
                                                gripper.fritction_cone_angle = M_PI/2.0;
                                                gripper.max_curvature = grasp_max_curvature;
                                                
                                                std::vector<int> grasp_indices;
                                                int jumps = 0;
                                                bool found = false;

                                                while(!found) {
                                                        gripper.find(grasper.points, grasp_indices,jumps);
                                                        if(grasp_indices.size()==0)break;

                                                        double x,y,z,roll,pitch,yaw;
                                                        gripper.getApproachRF(grasper.points, grasp_indices,x,y,z,roll,pitch,yaw);
                                                        z = slice->points[0].z;
                                                        Eigen::Matrix4d grasp_rf;
                                                        lar_tools::create_eigen_4x4_d(x,y,z,roll,pitch,yaw,grasp_rf);
                                                        Eigen::Matrix4d roty,rotz;
                                                        lar_tools::rotation_matrix_4x4_d('y',-M_PI/2.0,roty);
                                                        lar_tools::rotation_matrix_4x4_d('z',M_PI/2.0,rotz);
                                                        grasp_rf = grasp_rf*roty;
                                                        grasp_rf = grasp_rf*rotz;

                                                        Eigen::Vector3f base_x;
                                                        base_x<< 1,0,0;
                                                        Eigen::Vector3f approach_dir;
                                                        approach_dir<<
                                                        -grasp_rf(0,2),
                                                        -grasp_rf(1,2),
                                                        -grasp_rf(2,2);

                                                        double approach_angle_offset = acos(approach_dir.dot(base_x));

                                                          std::cout << "Approach Angle: "<<approach_angle_offset*180.0/M_PI<<std::endl;
                                                        if(fabs(approach_angle_offset)>M_PI/2.0){
                                                              jumps++;
                                                              continue;
                                                        }else{
                                                          found = true;
                                                        }

                                                        if (grasp_indices.size() > 0) {
                                                                pcl::PointCloud<PointType>::Ptr grasp_configuration_cloud(new pcl::PointCloud<PointType>());
                                                                pcl::PointCloud<PointType>::Ptr grasp_configuration_cloud_reprojection(new pcl::PointCloud<PointType>());
                                                                for (int i = 0; i < grasp_indices.size(); i++) {
                                                                        PointType p;
                                                                        p.x = grasper.points[grasp_indices[i]].p[0];
                                                                        p.y = grasper.points[grasp_indices[i]].p[1];
                                                                        p.z = slice->points[0].z;
                                                                        grasp_configuration_cloud->points.push_back(p);
                                                                }
                                                                pcl::transformPointCloud(*grasp_configuration_cloud, *grasp_configuration_cloud_reprojection, lar_tools::invert_transform_4x4(local_transform));
                                                                display_cloud(*viewer, grasp_configuration_cloud_reprojection, 0, 255, 0, 40, name + "_grasp_configuration");


                                                                draw_reference_frame(*viewer,  grasp_rf, 0.1f, name+"_approach_rf");
                                                        }
                                                }

                                        }
                                }
                        }



                }
        }

        //Draw

        display_cloud(*viewer, planes, 255, 0, 0, 4, "planes");

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

        viewer->addPointCloud(cloud_filtered,"ccc");
        viewer->addPointCloudNormals<PointType,NormalType>(cloud_filtered,cloud_normals);

        while (nh.ok() && !viewer->wasStopped()) {
                viewer->spinOnce();
                ros::spinOnce();
        }
}
