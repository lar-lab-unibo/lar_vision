
#include <ros/ros.h>


#include <cstdlib>
#include <chrono>

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

//Boost



#include "tsdf/tsdf_volume_octree.h"
#include "tsdf/marching_cubes_tsdf_octree.h"
#include "Noiser.h"

#define GRIPPER_STATUS_PARALLEL 0
#define GRIPPER_STATUS_TRIPOD 1
#define GRIPPER_STATUS_DUAL 2

using namespace std;
using namespace std::chrono;
using namespace lar_vision;

int width_ = 640;
int height_ = 480;
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
double th = 0.01f;

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
        lar_tools::init_ros_node(argc, argv, "tsdf_analyzer");
        ros::NodeHandle nh("~");

        //PCL
        pcl::visualization::PCLVisualizer* viewer = new pcl::visualization::PCLVisualizer("viewer");

        std::string source;
        nh.param<std::string>("source", source, "");
        nh.param<double>("th",th, 0.01);

        TSDFVolumeOctree::Ptr tsdf;
        tsdf.reset (new TSDFVolumeOctree);
        tsdf->load (source);

        /*
           MarchingCubesTSDFOctree mc;
           mc.setMinWeight (min_weight);

           high_resolution_clock::time_point t1 = high_resolution_clock::now();

           mc.setInputTSDF (tsdf);
           mc.setColorByRGB (true);

           pcl::PolygonMesh::Ptr mesh (new pcl::PolygonMesh);
           mc.reconstruct (*mesh);

           high_resolution_clock::time_point t2 = high_resolution_clock::now();
           auto duration = duration_cast<microseconds>( t2 - t1 ).count();
           std::cout << "Reconstruction duraton: "<<duration/1000<<" ms"<<std::endl;

           viewer->addPolygonMesh(*mesh,"meshes");
         */

        float val;
        pcl::PointXYZ pt;
        float max_val = std::numeric_limits<float>::min();
        float min_val = std::numeric_limits<float>::max();
        float max_w = std::numeric_limits<float>::min();
        float min_w = std::numeric_limits<float>::max();

        vector<RGBNode::Ptr> nodes_out;
        high_resolution_clock::time_point t1,t2;
        double duration;

        t1 = high_resolution_clock::now();
        tsdf->nodeList(nodes_out);
        t2 = high_resolution_clock::now();
        duration = duration_cast<microseconds>( t2 - t1 ).count()/1000.0f;
        std::cout << "Search Duration: "<<duration << " ms"<<std::endl;


        t1 = high_resolution_clock::now();
        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
        float x,y,z;
        float d,w;
        float color;
        for(int i = 0; i < nodes_out.size(); i++) {
                nodes_out[i]->getCenter (x,y,z);
                nodes_out[i]->getData(d,w);
                PointType pt;
                pt.x = x;
                pt.y = y;
                pt.z = z;
                pt.r = 0; pt.g = 0; pt.b = 0;
                if(d<=0.0f) {

                        nodes_out[i]->getRGB(
                          pt.r,pt.g,pt.b
                        );
                        cloud->points.push_back(pt);
                }
                /*if(d>=0.0f){

                   pt.r = (1.0f-d)*255;
                   }else{
                   pt.g = (1.0f-d)*255;
                   }*/



                max_val = d > max_val ? d : max_val;
                min_val = d < min_val ? d : min_val;
                max_w = d > max_w ? d : max_w;
                min_w = d < min_w ? d : min_w;
        }
        t2 = high_resolution_clock::now();
        duration = duration_cast<microseconds>( t2 - t1 ).count()/1000.0f;
        std::cout << "Simple Reconstruction Duration: "<<duration << " ms"<<std::endl;

        std::cout << "Nodes: "<<nodes_out.size()<<std::endl;
        std::cout << "Max val: "<<max_val<< " ,Min val: "<<min_val<<std::endl;
        std::cout << "Max W: "<<max_w<< " ,Min W: "<<min_w<<std::endl;

        viewer->addPointCloud(cloud,"cloud");
        while (nh.ok() && !viewer->wasStopped()) {

                ros::spinOnce();
                viewer->spinOnce();
        }

        return 0;
}
