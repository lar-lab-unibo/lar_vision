
#include <ros/ros.h>


#include <cstdlib>

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl/surface/concave_hull.h>
#include <pcl-1.8/pcl/impl/point_types.hpp>
#include <bits/stl_vector.h>


#include "lar_tools.h"
#include "lar_vision_commons.h"
#include "segmentation/HighMap.h"
#include "grasping/Slicer.h"
#include "grasping/Grasper.h"
#include "grasping/grippers/GraspingGripper.h"

#define GRIPPER_STATUS_PARALLEL 0
#define GRIPPER_STATUS_TRIPOD 1
#define GRIPPER_STATUS_DUAL 2

using namespace std;
using namespace lar_vision;

cv::Mat img;
std::vector<cv::Point2f> points;
cv::Point2f center;

std::string wname = "window";
double w;
double h;
double scale = 4000.0f;
double alpha = 0.006f;
double delta = 0.01f;
double eps = 0.005f;
int bypass = 0;
pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr hull(new pcl::PointCloud<PointType>);


int main(int argc, char** argv) {

    // Initialize ROS
    lar_tools::init_ros_node(argc, argv, "match_clouds");
    ros::NodeHandle nh("~");

    while (nh.ok()) {

        
    }

    return 0;
}
