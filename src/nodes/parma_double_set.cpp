#include <stdio.h>
#include <string.h>
#include <stdlib.h>

//ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <kdl/frames_io.hpp>
#include "geometry_msgs/Pose.h"

//OPENCV
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

//PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/shot_lrf.h>

//CUSTOM NODES
#include "lar_tools.h"
#include "commons/lar_vision_commons.h"

//boost
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>

//defines
#define TARGET_OBJECT_TYPE_HORIZONTAL 10001
#define TARGET_OBJECT_TYPE_VERTICAL 10003
#define GRASP_TYPE_PARALLEL 20001
#define GRASP_TYPE_TRIPOD 20002

using namespace std;

//ROS
ros::NodeHandle* nh;

typedef pcl::PointXYZ PointType;
typedef pcl::PointXYZRGB PointTypeColored;
typedef pcl::Normal NormalType;

//Pre Declaration
void computeRF(pcl::PointCloud<PointTypeColored>::Ptr& cloud, Eigen::Matrix4d& t);
void refineRF(Eigen::Matrix4d& rf, Eigen::Matrix4d& rf_refined, int& target_type);

//CLOUDS & VIEWER
pcl::visualization::PCLVisualizer* viewer;

//CLouds
pcl::PointCloud<PointType>::Ptr cloud_1(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud_trans_1(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud_trans_filtered_1(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud_2(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud_trans_2(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud_trans_filtered_2(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud_scene(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud_scene_boxed(new pcl::PointCloud<PointType>);


//filters
pcl::PassThrough<PointType> pass_filter;

//Topics
std::string depth_topic_1;
std::string depth_topic_2;
ros::Subscriber sub_cloud_1;
ros::Subscriber sub_cloud_2;
ros::Publisher bonmet_target_publisher;

geometry_msgs::PoseStamped bonmet_target_pose;


//Parameters

struct Box {
    double z_min;
    double z_max;
    double x_min;
    double x_max;
    double y_min;
    double y_max;
};
Box scene_bounding_box;
double noise_distance_mag = 0.1;
double target_approach_distance = 0.2;
double target_waypoint_mul = 1.0;
double target_waypoint_max_distance = 0.2;
int display_only_raw = 0;

/** TRANSFORMS */
Eigen::Matrix4d T_0_CAMERA_1;
Eigen::Matrix4d T_0_CAMERA_2;
Eigen::Matrix4d T_0_ROBOT;

int data_to_consume = 0;

//Sorter for Target Objects

struct TargetObject {
    pcl::PointCloud<PointTypeColored>::Ptr cloud;
    Eigen::Vector4f centroid;
    Eigen::Vector3f centroid3;
    Eigen::Matrix4d rf;
    Eigen::Matrix4d rf_refined;
    Eigen::Matrix4d rf_approach;
    int target_type;
    bool valid;

    TargetObject() {
        valid = false;
    }

    TargetObject(pcl::PointCloud<PointTypeColored>::Ptr& cloud_in) {
        cloud = cloud_in;
        pcl::compute3DCentroid(*cloud, centroid);
        for (int i = 0; i < 3; i++) {
            centroid3[i] = centroid[i];
        }
        computeRF(cloud, rf);
        refineRF(rf, rf_refined, target_type);
        Eigen::Matrix4d approach;
        lar_tools::create_eigen_4x4_d(0, 0, -target_approach_distance, 0, 0, 0, approach);
        rf_approach = rf_refined * approach;
        valid = true;
    }

    Eigen::Matrix4d getWaypoint(double mul) {
        if (target_type == TARGET_OBJECT_TYPE_HORIZONTAL) {
            Eigen::Matrix4d waypoint;
            lar_tools::create_eigen_4x4_d(0, 0, -target_waypoint_max_distance*mul, 0, 0, 0, waypoint);
            return rf_approach*waypoint;
        } else if (target_type == TARGET_OBJECT_TYPE_VERTICAL) {
            Eigen::Matrix4d waypoint;
            lar_tools::create_eigen_4x4_d(0, target_waypoint_max_distance*mul, -target_waypoint_max_distance * (mul * 0.1), 0, 0, 0, waypoint);
            return rf_approach*waypoint;
        }
    }


};

struct TargetObjectSorterX {

    inline bool operator()(const TargetObject& obj1, const TargetObject& obj2) {

        double x1 = obj1.centroid[0] + obj1.centroid[2]*10.0;
        double x2 = obj2.centroid[0] + obj2.centroid[2]*10.0;
        return x1 < x2;

    }
};

struct TargetObjectSorterZ {

    inline bool operator()(const TargetObject& obj1, const TargetObject& obj2) {
        return obj1.centroid[2] < obj2.centroid[2];
    }
};
std::vector<TargetObject> target_objects;
int selected_target_index = -1;
TargetObject selected_target_objects;

/**
 * Convert geometry_msgs::Pose to KDL::Frame
 * @param pose source geometry_msgs::Pose
 * @param frame target KDL::Frame
 */
void poseToFrame(const geometry_msgs::Pose& pose, KDL::Frame& frame) {
    KDL::Rotation r = KDL::Rotation::Quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
            );
    KDL::Vector p(
            pose.position.x,
            pose.position.y,
            pose.position.z
            );
    frame = KDL::Frame(r, p);
}

/**
 * Converts KDL::Frame to geometry_msgs::Pose
 * @param frame source KDL::Frame
 * @param pose target geometry_msgs::Pose
 */
void frameToPose(KDL::Frame& frame, geometry_msgs::Pose& pose) {
    pose.position.x = frame.p.x();
    pose.position.y = frame.p.y();
    pose.position.z = frame.p.z();

    frame.M.GetQuaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
            );
}

/**
 * KDL Frame to Eigen::Matrix4d
 */
void frameToEigen(KDL::Frame& frame, Eigen::Matrix4d& t) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            t(i, j) = frame.M(i, j);
        }
    }
    t(3, 0) = 0.0;
    t(3, 1) = 0.0;
    t(3, 2) = 0.0;
    t(3, 3) = 1.0;

    t(0, 3) = frame.p.x();
    t(1, 3) = frame.p.y();
    t(2, 3) = frame.p.z();
}

/**
 * Eigen::Matrix4d to  KDL Frame 
 * @param t
 * @param frame
 */
void eigenToFrame(Eigen::Matrix4d& t, KDL::Frame& frame) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            frame.M(i, j) = t(i, j);
        }
    }
    frame.p = KDL::Vector(t(0, 3), t(1, 3), t(2, 3));
}

void add_noise(pcl::PointCloud<PointType>::Ptr& cloud) {
    if (noise_distance_mag > 0.0001) {

        for (int i = 0; i < cloud->points.size(); i++) {
            PointType& p = cloud->points[i];
            p.z += (noise_distance_mag * p.z) * (double) (rand() % 100) / 100.0;
        }
    }

}

/**
 * Computes SHOT RF for a terget Cloud
 * @param cloud
 * @param t
 */
void computeRF(pcl::PointCloud<PointTypeColored>::Ptr& cloud, Eigen::Matrix4d& t) {
    if (cloud->points.size() <= 0) return;
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    pcl::PointCloud<PointTypeColored>::Ptr centroid_cloud(new pcl::PointCloud<PointTypeColored>());
    PointTypeColored p_centroid;
    p_centroid.x = centroid(0);
    p_centroid.y = centroid(1);
    p_centroid.z = centroid(2);
    centroid_cloud->points.push_back(p_centroid);

    pcl::PointCloud<pcl::ReferenceFrame>::Ptr rf_cloud(new pcl::PointCloud<pcl::ReferenceFrame>());


    pcl::SHOTLocalReferenceFrameEstimation<PointTypeColored, pcl::ReferenceFrame> est;
    est.setInputCloud(centroid_cloud);
    est.setSearchSurface(cloud);
    est.setRadiusSearch(20000.0f);
    est.compute(*rf_cloud);

    pcl::ReferenceFrame rf;
    rf = rf_cloud->points[0];

    t <<
            rf.x_axis[0], rf.y_axis[0], rf.z_axis[0], p_centroid.x,
            rf.x_axis[1], rf.y_axis[1], rf.z_axis[1], p_centroid.y,
            rf.x_axis[2], rf.y_axis[2], rf.z_axis[2], p_centroid.z,
            0, 0, 0, 1;

    if (t(0, 0) < 0) {
        Eigen::Matrix4d rotz;
        lar_tools::rotation_matrix_4x4_d('z', M_PI, rotz);
        t = t* rotz;
    }
    if (t(2, 2) < 0) {
        Eigen::Matrix4d rotx;
        lar_tools::rotation_matrix_4x4_d('x', M_PI, rotx);
        t = t* rotx;
    }
}

void extract_r_vector(Eigen::Matrix4d& rf, Eigen::Vector3d& v, char axis) {
    if (axis == 'z') {
        v << rf(0, 2), rf(1, 2), rf(2, 2);
    } else if (axis == 'y') {
        v << rf(0, 1), rf(1, 1), rf(2, 1);
    } else if (axis == 'x') {
        v << rf(0, 0), rf(1, 0), rf(2, 0);
    }
}

/**
 * Projects T Matrix on X-Y plane
 * @param rf
 * @param t_projected
 * @param up_axis
 */
void compute_projection(Eigen::Matrix4d& rf, Eigen::Matrix4d& t_projected, char up_axis = 'z') {

    Eigen::Vector3d x, y, z;
    Eigen::Matrix4d rotz, rotx, roty;

    x << 1, 0, 0;
    y << 0, 1, 0;
    z << 0, 0, 1;

    double beta_y = atan2(rf(1, 1), rf(0, 1));
    double beta_x = atan2(rf(1, 0), rf(0, 0));

    lar_tools::rotation_matrix_4x4_d('z', beta_y, rotz);

    Eigen::Vector4d old_y, old_x, new_x, new_y;
    old_y << y(0), y(1), y(2), 1;
    old_x << x(0), x(1), x(2), 1;
    new_y = rotz*old_y;
    new_x = rotz*old_x;

    t_projected(0, 0) = new_x(0);
    t_projected(1, 0) = new_x(1);
    t_projected(2, 0) = new_x(2);

    t_projected(0, 1) = new_y(0);
    t_projected(1, 1) = new_y(1);
    t_projected(2, 1) = new_y(2);

    t_projected(0, 2) = 0;
    t_projected(1, 2) = 0;
    t_projected(2, 2) = 1;

    Eigen::Matrix4d rot;
    if (up_axis == 'z') {

    } else if (up_axis == 'x') {
        lar_tools::rotation_matrix_4x4_d('y', M_PI / 2.0, rot);
        t_projected = t_projected * rot;
    } else if (up_axis == 'y') {

    }
}

/**
 * Refines RF for custom task
 * @param rf
 * @param rf_refined
 */
void refineRF(Eigen::Matrix4d& rf, Eigen::Matrix4d& rf_refined, int& target_type) {

    Eigen::Matrix4d rotz, rotx, roty;

    Eigen::Vector3d x, y, z;
    x << 1, 0, 0;
    y << 0, 1, 0;
    z << 0, 0, 1;

    Eigen::Vector3d rf_x, rf_y, rf_z;

    rf_z << rf(0, 2), rf(1, 2), rf(2, 2);
    rf_x << rf(0, 0), rf(1, 0), rf(2, 0);

    double magz = rf_z.dot(z);
    double magx = rf_x.dot(z);

    rf_refined = rf;

    if (fabs(magx) > fabs(magz)) {

        //OBJECT IS ORTHOGONAL TO THE GROUND
        target_type = TARGET_OBJECT_TYPE_VERTICAL;
        compute_projection(rf_refined, rf_refined, 'z');
        lar_tools::rotation_matrix_4x4_d('x', M_PI / 2.0, rotx);
        rf_refined = rf_refined * rotx;
        Eigen::Vector3d new_z;
        extract_r_vector(rf_refined, new_z, 'z');
        if (new_z.dot(x) < 0) {
            lar_tools::rotation_matrix_4x4_d('y', M_PI, roty);
            rf_refined = rf_refined * roty;
        }


    } else {

        //OBJECT IS PARALLEL TO THE GROUND
        target_type = TARGET_OBJECT_TYPE_HORIZONTAL;
        compute_projection(rf_refined, rf_refined, 'z');

        Eigen::Vector3d new_z;
        extract_r_vector(rf_refined, new_z, 'z');
        if (new_z.dot(z) > 0) {
            lar_tools::rotation_matrix_4x4_d('y', M_PI, roty);
            rf_refined = rf_refined * roty;
        }

    }

}

/**
 * Clusterizes scene
 */
void build_clusters() {

    pcl::search::KdTree<PointType>::Ptr tree2(new pcl::search::KdTree<PointType>);
    tree2->setInputCloud(cloud_scene_boxed);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointType> ec;
    ec.setClusterTolerance(0.02);
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree2);
    ec.setInputCloud(cloud_scene_boxed);
    ec.extract(cluster_indices);

    target_objects.clear();
    for (int i = 0; i < cluster_indices.size(); i++) {
        pcl::PointCloud<PointTypeColored>::Ptr cluster(new pcl::PointCloud<PointTypeColored>);
        pcl::copyPointCloud(*cloud_scene_boxed, cluster_indices[i].indices, *cluster);
        target_objects.push_back(TargetObject(cluster));
    }
    std::sort(target_objects.begin(), target_objects.end(), TargetObjectSorterX());

}

/**
 * Space Boxing with limits on X,Y,Z
 * @param cloud_in
 * @param bounding_box
 * @param cloud_out
 */
void filter_box_cloud(pcl::PointCloud<PointType>::Ptr cloud_in,
        Box bounding_box,
        pcl::PointCloud<PointType>::Ptr cloud_out
        ) {

    //Z
    pass_filter.setInputCloud(cloud_in);
    pass_filter.setFilterFieldName("z");
    pass_filter.setFilterLimits(bounding_box.z_min, bounding_box.z_max);
    pass_filter.filter(*cloud_out);

    //X
    pass_filter.setInputCloud(cloud_out);
    pass_filter.setFilterFieldName("x");
    pass_filter.setFilterLimits(bounding_box.x_min, bounding_box.x_max);
    pass_filter.filter(*cloud_out);

    //Y
    pass_filter.setInputCloud(cloud_out);
    pass_filter.setFilterFieldName("y");
    pass_filter.setFilterLimits(bounding_box.y_min, bounding_box.y_max);
    pass_filter.filter(*cloud_out);

}

/**
 * Callbacks for Camera 1
 * @param input
 */
void cloud_1_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
    //if(cloud_consuming) return;
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(*input, pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc, *cloud_1);

    add_noise(cloud_1);
    pcl::transformPointCloud(*cloud_1, *cloud_trans_1, T_0_CAMERA_1);

}

/**
 * Callbacks for Camera 2
 * @param input
 */
void cloud_2_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
    //if(cloud_consuming) return;
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(*input, pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc, *cloud_2);

    add_noise(cloud_2);
    pcl::transformPointCloud(*cloud_2, *cloud_trans_2, T_0_CAMERA_2);

}

/**
 * Builds scene merging camera frames
 */
void build_scene() {

    cloud_scene = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);

    (*cloud_scene) += (*cloud_trans_1);
    (*cloud_scene) += (*cloud_trans_2);

    filter_box_cloud(cloud_scene, scene_bounding_box, cloud_scene_boxed);
}

/**
 * Target Object selection
 */
void select_target(bool reset = false) {

    if (reset) {
        selected_target_index = -1;
    } else {
        selected_target_index++;
        if (selected_target_index > target_objects.size() - 1) {
            selected_target_index = -1;
        }
    }

    if (selected_target_index >= 0) {
        selected_target_objects = target_objects[selected_target_index];

    }
}

/**
 * Adjusts global Waypoint multiplier
 * @param d
 */
void adjust_multiplier(double d) {
    target_waypoint_mul += d;
    if (target_waypoint_mul > 1.0)target_waypoint_mul = 1.0;
    if (target_waypoint_mul < 0.0)target_waypoint_mul = 0.0;
    ROS_INFO("Waypoint multiplier: %f", target_waypoint_mul);
}

/**
 * 3D Viewer Kayboard callbacks
 * @param event
 * @param viewer_void
 */
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
        void* viewer_void) {
    if (event.getKeySym() == "a" && event.keyDown()) {
        select_target();
    }
    if (event.getKeySym() == "x" && event.keyDown()) {
        select_target(true);
    }
    if (event.getKeySym() == "m" && event.keyDown()) {
        adjust_multiplier(0.1);
    }
    if (event.getKeySym() == "n" && event.keyDown()) {
        adjust_multiplier(-0.1);
    }
}

/**
 * Visualization Part
 */
void visualize() {
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    viewer->addPointCloud(cloud_scene_boxed, "scene");

    lar_vision::Palette palette;

    std::stringstream ss;

    if (display_only_raw > 0) {
        viewer->addPointCloud(cloud_trans_1, "cam_1_cloud");
        viewer->addPointCloud(cloud_trans_2, "cam_2_cloud");
        return;
    }
    
    for (int i = 0; i < target_objects.size(); +i++) {
        Eigen::Vector3i color = palette.getColor();
        ss.str("");
        ss << "target_object_" << i;
        lar_vision::display_cloud(*viewer, target_objects[i].cloud, color[0], color[1], color[2], 2.0, ss.str());
        ss.str("");
        ss << "target_" << i;


        if (target_objects[i].target_type == TARGET_OBJECT_TYPE_HORIZONTAL) {
            ss << "_horizontal";
        } else if (target_objects[i].target_type == TARGET_OBJECT_TYPE_VERTICAL) {
            ss << "_vertical";
        }
        lar_vision::draw_text_3D(*viewer, ss.str(), target_objects[i].centroid3, 255, 255, 255, 0.02, ss.str());
        ss << "_rf";
        lar_vision::draw_reference_frame(*viewer, target_objects[i].rf, 0.03f, ss.str());
        ss << "_rf_ref";
        lar_vision::draw_reference_frame(*viewer, target_objects[i].rf_refined, 0.1f, ss.str());
        ss << "_rf_ref_approach";
        lar_vision::draw_reference_frame(*viewer, target_objects[i].rf_approach, 0.12f, ss.str());


    }

    //Selected object
    if (selected_target_index >= 0) {
        lar_vision::display_cloud(*viewer, selected_target_objects.cloud, 255, 255, 255, 8.0, "selected_object");
        Eigen::Matrix4d waypoint = selected_target_objects.getWaypoint(target_waypoint_mul);
        lar_vision::draw_reference_frame(*viewer, waypoint, 0.2f, "selected_object_waypoint");
    }
}

/** MAIN NODE **/
int main(int argc, char** argv) {

    // Initialize ROS
    ros::init(argc, argv, "parma_double_set");
    ROS_INFO("parma_double_set node started...");
    nh = new ros::NodeHandle("~");
    
    
    double cam_1_x,cam_1_y,cam_1_z;
    double cam_2_x,cam_2_y,cam_2_z;
    
    nh->param<std::string>("depth_topic_1", depth_topic_1, "/vrep/camera_depth_1");
    nh->param<std::string>("depth_topic_2", depth_topic_2, "/vrep/camera_depth_2");
    nh->param<double>("bounding_box_x_min", scene_bounding_box.x_min, 0.0);
    nh->param<double>("bounding_box_x_max", scene_bounding_box.x_max, 1.0);
    nh->param<double>("bounding_box_y_min", scene_bounding_box.y_min, -0.5);
    nh->param<double>("bounding_box_y_max", scene_bounding_box.y_max, 0.5);
    nh->param<double>("bounding_box_z_min", scene_bounding_box.z_min, 0.01);
    nh->param<double>("bounding_box_z_max", scene_bounding_box.z_max, 2.0);
    nh->param<double>("noise_mag", noise_distance_mag, 0.01);
    nh->param<double>("target_approach_distance", target_approach_distance, 0.2);
    nh->param<int>("display_only_raw", display_only_raw, 0);
    
    nh->param<double>("cam_1_x", cam_1_x, 0.38);
    nh->param<double>("cam_1_y", cam_1_y, 1.03);
    nh->param<double>("cam_1_z", cam_1_z, 0.35);
    
    nh->param<double>("cam_2_x", cam_2_x, 0.40);
    nh->param<double>("cam_2_y", cam_2_y, -0.6);
    nh->param<double>("cam_2_z", cam_2_z, 0.25);


    /** VIEWER */
    viewer = new pcl::visualization::PCLVisualizer("viewer");
    viewer->registerKeyboardCallback(keyboardEventOccurred, (void*) &viewer);

    /** TRANSFORMS */
    //VREP
    //    lar_tools::create_eigen_4x4_d(0, 0, 0, 0, 0, 0, T_0_ROBOT);
    //    lar_tools::create_eigen_4x4_d(0.5, -0.684, 0.616, 0, 0, 0, T_0_CAMERA_1);
    //    lar_tools::create_eigen_4x4_d(0.5, 0.959, 0.617, 0, 0, 0, T_0_CAMERA_2);
    //    Eigen::Matrix4d rotx, rotz;
    //    lar_tools::rotation_matrix_4x4_d('x', M_PI / 2.0 + M_PI / 6.0, rotx);
    //    lar_tools::rotation_matrix_4x4_d('z', M_PI, rotz);
    //    T_0_CAMERA_2 = T_0_CAMERA_2 * rotx;
    //    T_0_CAMERA_1 = T_0_CAMERA_1 * rotz;
    //    T_0_CAMERA_1 = T_0_CAMERA_1 * rotx;

    //Test Setup
    lar_tools::create_eigen_4x4_d(0, 0, 0, 0, 0, 0, T_0_ROBOT);
    lar_tools::create_eigen_4x4_d(cam_1_x, cam_1_y, cam_1_z, 0, 0, 0, T_0_CAMERA_1);
    lar_tools::create_eigen_4x4_d(cam_2_x, cam_2_y, cam_2_z, 0, 0, 0, T_0_CAMERA_2);

    Eigen::Matrix4d rotx, rotz;
    lar_tools::rotation_matrix_4x4_d('x', -M_PI / 2.0, rotx);
    lar_tools::rotation_matrix_4x4_d('z', M_PI, rotz);
    T_0_CAMERA_2 = T_0_CAMERA_2 * rotx;
    T_0_CAMERA_1 = T_0_CAMERA_1 * rotz;
    T_0_CAMERA_1 = T_0_CAMERA_1 * rotx;

    //Topics Subscription/Advertising
    sub_cloud_1 = nh->subscribe(depth_topic_1, 1, cloud_1_cb);
    sub_cloud_2 = nh->subscribe(depth_topic_2, 1, cloud_2_cb);
    bonmet_target_publisher = nh->advertise<geometry_msgs::PoseStamped>("/bonmetc60/target_ik", 1);

    // Spin & Time
    ros::Rate r(10);
    // Spin
    while (nh->ok() && !viewer->wasStopped()) {

        build_scene();
        build_clusters();
        visualize();


        if (selected_target_index >= 0) {
            KDL::Frame target_frame;
            Eigen::Matrix4d waypoint = selected_target_objects.getWaypoint(target_waypoint_mul);
            eigenToFrame(waypoint, target_frame);
            frameToPose(target_frame, bonmet_target_pose.pose);
            bonmet_target_publisher.publish(bonmet_target_pose);
        }

        //viewer->addPointCloud(cloud_trans_2,"cloud_2");
        lar_vision::draw_reference_frame(*viewer, T_0_ROBOT, 1.0, "T_0_ROBOT");

        viewer->spinOnce();
        ros::spinOnce();
        r.sleep();
    }

}
