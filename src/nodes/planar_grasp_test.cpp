
#include <ros/ros.h>


#include <cstdlib>

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/impl/point_types.hpp>
#include <bits/stl_vector.h>


#include "lar_tools.h"
#include "lar_vision_commons.h"
#include "segmentation/HighMap.h"
#include "grasping/Slicer.h"
#include "grasping/Grasper.h"
#include "grasping/grippers/CrabbyGripper.h"

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
bool discard_invalids = false;
double grasp_min_offset = 0.01;
double grasp_max_offset = 0.2;
double grasp_min_radius = 0.01;
double grasp_max_radius = 0.05;
double grasp_fritction_cone_angle = 1.57;
double grasp_max_curvature = 1.57;
int grasp_type = GRIPPER_STATUS_PARALLEL;

int bypass = 0;
pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr hull(new pcl::PointCloud<PointType>);

std::vector<cv::Point2f> refined_points;
std::vector<cv::Point2f> normals;

void pointsToCloud(std::vector<cv::Point2f>& points, pcl::PointCloud<PointType>::Ptr& cloud, bool reverse = false) {
    cloud->points.clear();
    for (int i = 0; i < points.size(); i++) {
        PointType pt;
        pt.x = points[i].x / scale;
        pt.y = points[i].y / scale;
        cloud->points.push_back(pt);
    }
}

void drawArrow(cv::Mat& img, cv::Point2f p, cv::Point2f q, cv::Scalar color, int thickness = 1, int arrowMagnitude = 19, int line_type = 8, int shift = 0) {
    //Draw the principle line
    cv::line(img, p, q, color, thickness);
    const double PI = 3.141592653;
    //compute the angle alpha
    double angle = atan2((double) p.y - q.y, (double) p.x - q.x);
    //compute the coordinates of the first segment
    p.x = (int) (q.x + arrowMagnitude * cos(angle + PI / 4));
    p.y = (int) (q.y + arrowMagnitude * sin(angle + PI / 4));
    //Draw the first segment
    cv::line(img, p, q, color, thickness);
    //compute the coordinates of the second segment
    p.x = (int) (q.x + arrowMagnitude * cos(angle - PI / 4));
    p.y = (int) (q.y + arrowMagnitude * sin(angle - PI / 4));
    //Draw the second segment
    cv::line(img, p, q, color, thickness);
}

void update() {

    img = cv::Mat::zeros(h, w, CV_32FC3);
    pointsToCloud(points, cloud);

    Grasper grasper(alpha, delta);
    grasper.setCloud(cloud);

    for (int i = 0; i < points.size(); i++) {
        cv::circle(img, points[i], 5.0f, cv::Scalar(0, 0, 255), -1);
    }


    for (int i = 0; i < grasper.points.size(); i++) {
        cv::Point2f p(grasper.points[i].p(0) * scale, grasper.points[i].p(1) * scale);
        cv::circle(img, p, 3.0f, cv::Scalar(255, 255, 255), -1);
    }
    cv::circle(img, cv::Point2f(grasper.centroid(0) * scale, grasper.centroid(1) * scale), 10.0f, cv::Scalar(0, 0, 255), 3);


    CrabbyGripper gripper(grasp_type);

    gripper.auto_discard_planar_invalids = discard_invalids;
    gripper.min_offset = grasp_min_offset;
    gripper.max_offset = grasp_max_offset;
    gripper.min_radius = grasp_min_radius;
    gripper.max_radius = grasp_max_radius;
    gripper.fritction_cone_angle = grasp_fritction_cone_angle;
    gripper.max_curvature = grasp_max_curvature;

    std::vector<int> grasp_indices;
    gripper.find(grasper.points, grasp_indices, bypass);
    bool valid = gripper.isValidPlanarConfiguration(grasper.points,grasp_indices);

    for (int i = 0; i < grasp_indices.size(); i++) {
        cv::Point2f point(grasper.points[grasp_indices[i]].p(0) * scale, grasper.points[grasp_indices[i]].p(1) * scale);
        cv::Point2f normal(grasper.points[grasp_indices[i]].normal(0), grasper.points[grasp_indices[i]].normal(1));

        if (i == 2) {
            cv::circle(img, point, 15.0f, cv::Scalar(255, 0, 255), 4);
            if (valid) {
                drawArrow(img, point, point + normal * 100, cv::Scalar(0, 255, 0), 2);
            }else{
                drawArrow(img, point, point + normal * 100, cv::Scalar(0, 0, 255), 2);
            }
        } else {
            cv::circle(img, point, 15.0f, cv::Scalar(0, 255, 255), 4);
            drawArrow(img, point, point + normal * 100, cv::Scalar(0, 0, 255), 2);
        }
    }


    cv::imshow(wname, img);
}

void CallBackFunc(int event, int x, int y, int flags, void* userdata) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
        cv::Point2f p(x, y);
        points.push_back(p);
        bypass = 0;
        update();
    }
}

int main(int argc, char** argv) {

    // Initialize ROS
    lar_tools::init_ros_node(argc, argv, "planar_grasp_test");
    ros::NodeHandle nh("~");

    nh.param<double>("w", w, 800);
    nh.param<double>("h", h, 800);
    nh.param<double>("alpha", alpha, 0.1f);
    nh.param<double>("delta", delta, 0.01f);
    nh.param<double>("eps", eps, 0.005f);
    nh.param<bool>("discard_invalids", discard_invalids, false);
    nh.param<int>("grasp_type", grasp_type, GRIPPER_STATUS_PARALLEL);

    nh.param<double>("grasp_min_offset",grasp_min_offset,0.01);
    nh.param<double>("grasp_max_offset",grasp_max_offset,0.2);
    nh.param<double>("grasp_min_radius",grasp_min_radius,0.01);
    nh.param<double>("grasp_max_radius",grasp_max_radius,0.05);
    nh.param<double>("grasp_fritction_cone_angle",grasp_fritction_cone_angle,M_PI/2.0);
    nh.param<double>("grasp_max_curvature",grasp_max_curvature,1);



    /* VIEWER */
    //    viewer = new pcl::visualization::PCLVisualizer("Bunch Tester Viewer");
    //    viewer->registerKeyboardCallback(keyboardEventOccurred, (void*) &viewer);

    img = cv::Mat::zeros(h, w, CV_32FC3);


    //Create a windowù

    cv::namedWindow(wname, 1);
    cv::setMouseCallback(wname, CallBackFunc, NULL);
    cv::imshow(wname, img);

    short c = cv::waitKey(10);
    while (true) {

        c = cv::waitKey(10);
        if (c != -1) {
            if (c == 113) {
                break;
            }
            if (c == 115) {
                alpha -= 0.001f;
                std::cout << "Alpha: " << alpha << std::endl;
                update();
            }
            if (c == 100) {
                alpha += 0.001f;
                std::cout << "Alpha: " << alpha << std::endl;
                update();
            }
            if (c == 98) {
                bypass++;

                update();
            }
            std::cout << c << std::endl;
        }
    }

    return 0;
}
