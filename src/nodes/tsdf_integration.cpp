
#include <ros/ros.h>


#include <cstdlib>

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/surface/concave_hull.h>
#include <bits/stl_vector.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/passthrough.h>

#include "lar_tools.h"
#include "lar_vision_commons.h"
#include "segmentation/HighMap.h"
#include "grasping/Slicer.h"
#include "grasping/Grasper.h"
#include "grasping/grippers/GraspingGripper.h"


#include "tsdf/tsdf_volume_octree.h"
#include "tsdf/marching_cubes_tsdf_octree.h"
#include "Noiser.h"

#define GRIPPER_STATUS_PARALLEL 0
#define GRIPPER_STATUS_TRIPOD 1
#define GRIPPER_STATUS_DUAL 2

using namespace std;
using namespace lar_vision;

int width_ = 640;
int height_ = 480;
int crop_width = width_;
int crop_height = height_;
double offx = 0;
double offy = 0;
double offz = 0;
double focal_length_x_ = 525.;
double focal_length_y_ = 525.;
double principal_point_x_ = 319.5;
double principal_point_y_ = 239.5;
double tsdf_size = 2;
double tsdf_res;
double cell_size = 0.005;
int num_random_splits = 1;
double min_sensor_dist = 0.0;
double max_sensor_dist = 3.0;
double integrate_color = true;
double trunc_dist_pos = 0.01;
double trunc_dist_neg = 0.01;
double min_weight = 0;
bool cloud_noise = true;
bool camera_noise = true;
bool do_simple_merge = false;
double leaf = 0.01f;

double camera_error_x = 0;
double camera_error_y = 0;
double camera_error_z = 0;
double camera_error_roll = 0;
double camera_error_pitch = 0;
double camera_error_yaw = 0;

int frame_jumps = 1;
bool more_noise = false;

int main(int argc, char** argv) {

        // Initialize ROS
        lar_tools::init_ros_node(argc, argv, "tsdf_integration");
        ros::NodeHandle nh("~");

        std::string out_dir = "~/temp/";

        std::string path = "~/temp/temp_clouds/";
        std::string output_name = "merge_results";

        nh.param<std::string>("folder", path, "~/temp/temp_clouds/");
        nh.param<std::string>("output_name", output_name,"merge_results");
        nh.param<std::string>("out_folder", out_dir,"~/temp/");
        nh.param<bool>("simple_merge", do_simple_merge, false);
        nh.param<bool>("cloud_noise", cloud_noise, false);
        nh.param<bool>("camera_noise", camera_noise, false);
        nh.param<bool>("more_noise", more_noise, false);


        nh.param<double>("camera_error_x", camera_error_x, 0.0);
        nh.param<double>("camera_error_y", camera_error_y, 0.0);
        nh.param<double>("camera_error_z", camera_error_z, 0.0);
        nh.param<double>("camera_error_roll", camera_error_roll, 0.0);
        nh.param<double>("camera_error_pitch", camera_error_pitch, 0.0);
        nh.param<double>("camera_error_yaw", camera_error_yaw, 0.0);


        nh.param<double>("tsdf_size", tsdf_size, 1.0);
        nh.param<double>("cell_size", cell_size, 0.01);
        nh.param<double>("trunc_dist_pos", trunc_dist_pos, 0.03);
        nh.param<double>("trunc_dist_neg", trunc_dist_neg, 0.03);
        nh.param<double>("max_sensor_dist", max_sensor_dist, 3.0);
        nh.param<double>("min_sensor_dist", min_sensor_dist, 0.0);
        nh.param<double>("min_weight", min_weight, 0.0);
        nh.param<int>("crop_width", crop_width, width_);
        nh.param<int>("crop_height", crop_height, height_);

        nh.param<double>("fx", focal_length_x_, 525.);
        nh.param<double>("fy", focal_length_y_, 525.);
        nh.param<double>("cx", principal_point_x_, 319.5);
        nh.param<double>("cy", principal_point_y_, 239.5);

        nh.param<double>("offx", offx, 0.0);
        nh.param<double>("offy", offy, 0.0);
        nh.param<double>("offz", offz, 0.0);
        nh.param<double>("leaf", leaf, 0.01);
        nh.param<int>("frame_jumps", frame_jumps, 1);
        int total_perc_n = 100/frame_jumps;
        std::string total_perc = boost::lexical_cast<std::string>(total_perc_n);

        std::string cloud_suffix = cloud_noise ? "_noise" : "";

        Eigen::Matrix4d camera_error_matrix;
        lar_tools::create_eigen_4x4_d(
                (float)camera_error_x,
                (float)camera_error_y,
                (float)camera_error_z,
                (float)camera_error_roll,
                (float)camera_error_pitch,
                (float)camera_error_yaw,
                camera_error_matrix
                );

        if(!do_simple_merge) {
                int desired_res = tsdf_size / cell_size;
                int n = 1;
                while (desired_res > n) n *= 2;
                tsdf_res = n;


                TSDFVolumeOctree::Ptr tsdf;
                tsdf.reset (new TSDFVolumeOctree);
                tsdf->setGridSize (tsdf_size, tsdf_size, tsdf_size);
                ROS_INFO("Setting resolution: %f with grid size %f and voxel size %f\n", tsdf_res, tsdf_size,cell_size);
                tsdf->setResolution (tsdf_res, tsdf_res, tsdf_res);
                tsdf->setImageSize (width_, height_);
                tsdf->setImageCropSize(crop_width,crop_height);
                tsdf->setCameraIntrinsics (focal_length_x_, focal_length_y_, principal_point_x_, principal_point_y_);
                tsdf->setNumRandomSplts (num_random_splits);
                tsdf->setSensorDistanceBounds (min_sensor_dist, max_sensor_dist);
                tsdf->setIntegrateColor (integrate_color);
                tsdf->setDepthTruncationLimits (trunc_dist_pos, trunc_dist_neg);
                tsdf->reset ();





                ROS_INFO("Start integration...");
                Eigen::Matrix4d pose_0;
                int i = 0;
                for(;; ) {


                        std::string pose_filename = path + boost::lexical_cast<std::string>(i) +".txt";
                        std::string pose_robot_filename = path + boost::lexical_cast<std::string>(i) + "_robot.txt";
                        std::string pose_ee_filename = path + boost::lexical_cast<std::string>(i) + "_ee.txt";
                        std::string cloud_filename = path + boost::lexical_cast<std::string>(i) +cloud_suffix+ ".pcd";

                        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
                        pcl::PointCloud<PointType>::Ptr cloud_reduced(new pcl::PointCloud<PointType>());
                        pcl::PointCloud<PointType>::Ptr cloud_trans(new pcl::PointCloud<PointType>());
                        Eigen::Matrix4d pose = lar_tools::load_transform_4x4_d(pose_filename);
                        Eigen::Matrix4d pose_robot= lar_tools::load_transform_4x4_d(pose_robot_filename);;
                        Eigen::Matrix4d pose_ee= lar_tools::load_transform_4x4_d(pose_ee_filename);;

                        if(camera_noise) {
                                std::cout << "Pose error:\n"<<camera_error_matrix<<std::endl;
                                pose_ee = pose_ee * camera_error_matrix;

                                pose = pose_robot * pose_ee;
                        }


                        if(i==0) {
                                pose_0 = pose;
                        }
                        Eigen::Affine3d tr;
                        tr  =   pose_0.inverse () * pose;

                        std::cout << "Loading Cloud\n";
                        if(pcl::io::loadPCDFile (cloud_filename, *cloud)==-1) {
                                break;
                        }

                        //MORE NOISE
                        if(more_noise) {
                                Noiser noiser;
                                noiser.axial_coefficient=0.1;
                                noiser.setCloud(cloud);
                                pcl::PointCloud<PointType>::Ptr cloud_noise(new pcl::PointCloud<PointType>());
                                noiser.addNoise(cloud_noise);
                                cloud=cloud_noise;
                        }

                        cloud->width = width_;
                        cloud->height = height_;

                        if (cloud->width != width_ || cloud->height != height_)
                        {
                                PCL_ERROR ("Error: cloud %d has size %d x %d, but TSDF is initialized for %d x %d pointclouds\n", i+1, cloud->width, cloud->height, width_, height_);
                                return (1);
                        }


                        tsdf->integrateCloud (*cloud, pcl::PointCloud<pcl::Normal> (),tr);

                        std::cout << pose_filename << "\n"<<cloud_filename<<"\n\n";
                        i+=frame_jumps;
                }





                MarchingCubesTSDFOctree mc;
                mc.setMinWeight (min_weight);
                mc.setInputTSDF (tsdf);
                mc.setColorByRGB (true);

                pcl::PolygonMesh::Ptr mesh (new pcl::PolygonMesh);
                mc.reconstruct (*mesh);
//        if (flatten)
//              flattenVertices (*mesh);
//    if (cleanup)
//          cleanupMesh (*mesh);
//if (visualize)
//{
//      vis->removeAllPointClouds ();
//    vis->addPolygonMesh (*mesh);
//  vis->spin ();
                //}
                //PCL_INFO ("Entire pipeline took %f ms\n", tt.toc ());
                //if (save_ascii)
                //      pcl::io::savePLYFile (out_dir + "/mesh.ply", *mesh);
                //else

                pcl::PointCloud<PointType>::Ptr cloud_out(new pcl::PointCloud<PointType>);
                pcl::fromPCLPointCloud2(mesh->cloud, *cloud_out);
                pcl::transformPointCloud(*cloud_out,*cloud_out,pose_0);

                std::string pcd_out_filename = out_dir + "/" +output_name+"_tsdf_"+total_perc+cloud_suffix+".pcd";
                std::string ply_out_filename = out_dir + "/" +output_name+"_tsdf_mesh_"+total_perc+cloud_suffix+".ply";
                std::string tsdf_out_filename = out_dir + "/" +output_name+"_tsdf_mesh_"+total_perc+cloud_suffix+".tsdf";

                ROS_INFO("Writing PCD: %s",pcd_out_filename.c_str());
                ROS_INFO("Writing PLY: %s",ply_out_filename.c_str());
                ROS_INFO("Writing TSDF: %s",tsdf_out_filename.c_str());

                pcl::io::savePCDFileBinary(pcd_out_filename, *cloud_out);
                pcl::io::savePLYFileBinary(ply_out_filename, *mesh);
                tsdf->save(tsdf_out_filename);

        }else{
                pcl::PointCloud<PointType>::Ptr full_filtered(new pcl::PointCloud<PointType>());
                int i = 0;
                for(;; ) {

                        
                        std::string pose_filename = path + boost::lexical_cast<std::string>(i) +".txt";
                        std::string pose_robot_filename = path + boost::lexical_cast<std::string>(i) + "_robot.txt";
                        std::string pose_ee_filename = path + boost::lexical_cast<std::string>(i) + "_ee.txt";
                        std::string cloud_filename = path + boost::lexical_cast<std::string>(i) +cloud_suffix+ ".pcd";

                        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
                        pcl::PointCloud<PointType>::Ptr cloud_trans(new pcl::PointCloud<PointType>());



                        //pose = lar_tools::load_transform_4x4_d(pose_filename);

                        std::cout << "Loading Cloud\n";
                        if(pcl::io::loadPCDFile (cloud_filename, *cloud)==-1) {
                                break;
                        }

                        std::cout << "Loading Pose\n";
                        Eigen::Matrix4d pose = lar_tools::load_transform_4x4_d(pose_filename);
                        Eigen::Matrix4d pose_robot= lar_tools::load_transform_4x4_d(pose_robot_filename);;
                        Eigen::Matrix4d pose_ee= lar_tools::load_transform_4x4_d(pose_ee_filename);;

                        if(camera_noise) {
                                std::cout << "Pose error:\n"<<camera_error_matrix<<std::endl;
                                pose_ee = pose_ee * camera_error_matrix;

                                pose = pose_robot * pose_ee;
                        }


                        pcl::PassThrough<PointType> pass;
                        pass.setInputCloud (cloud);
                        pass.setFilterFieldName ("z");
                        pass.setFilterLimits (min_sensor_dist,max_sensor_dist);
                        pass.filter (*cloud);

                        if(more_noise) {
                                Noiser noiser;
                                noiser.axial_coefficient=0.1;
                                noiser.axial_offset =0;
                                noiser.setCloud(cloud);
                                pcl::PointCloud<PointType>::Ptr cloud_noise(new pcl::PointCloud<PointType>());
                                noiser.addNoise(cloud_noise);
                                cloud=cloud_noise;
                        }

                        pcl::transformPointCloud(*cloud, *cloud_trans, pose);
                        (*full_filtered) += (*cloud_trans);

                        pcl::VoxelGrid<PointType> sor;
                        sor.setInputCloud (full_filtered);
                        sor.setLeafSize (leaf,leaf,leaf);
                        sor.filter (*full_filtered);

                        i+=frame_jumps;
                }

                pcl::io::savePCDFileBinary(out_dir + "/" +output_name+"_merge_"+total_perc+cloud_suffix+".pcd", *full_filtered);


        }
        //while (nh.ok()) {
        ros::spinOnce();
        //}

        return 0;
}
