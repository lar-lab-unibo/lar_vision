/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   lar_vision_commons.h
 * Author: daniele
 *
 * Created on November 9, 2015, 12:00 PM
 */
#include "lar_tools.h"
#include "lar_vision_commons.h"

#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_lrf.h>

namespace lar_vision {

    void
    display_cloud(pcl::visualization::PCLVisualizer &viewer, pcl::PointCloud<PointType>::Ptr& cloud, int r, int g, int b, int size, std::string name) {
        pcl::visualization::PointCloudColorHandlerCustom<PointType> cloud_color(cloud, r, g, b);
        viewer.addPointCloud(cloud, cloud_color, name);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, name);
    }

    void
    compute_normals(pcl::PointCloud<PointType>::Ptr& cloud, pcl::PointCloud<NormalType>::Ptr& cloud_normals, float radius_search) {
        pcl::NormalEstimationOMP<PointType, NormalType> ne;
        ne.setInputCloud(cloud);
        pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType> ());
        ne.setSearchMethod(tree);
        ne.setRadiusSearch(radius_search);
        ne.compute(*cloud_normals);

        //        pcl::IntegralImageNormalEstimation<PointType, NormalType> ne;
        //        ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
        //        ne.setMaxDepthChangeFactor(0.02f);
        //        ne.setNormalSmoothingSize(10.0f);
        //        ne.setInputCloud(cloud);
        //        ne.compute(*cloud_normals);
    }

    void
    convert_point_3D(PointType& pt, Eigen::Vector3f& p, bool reverse) {
        if (!reverse) {
            p[0] = pt.x;
            p[1] = pt.y;
            p[2] = pt.z;
        } else {
            pt.x = p[0];
            pt.y = p[1];
            pt.z = p[2];
        }
    }

    void
    draw_3D_vector(pcl::visualization::PCLVisualizer& viewer, Eigen::Vector3f start, Eigen::Vector3f end, float r, float g, float b, std::string name) {
        PointType p_start, p_end;
        convert_point_3D(p_start, start, true);

        Eigen::Vector3f dv;
        dv = end - start;

        p_end.x = p_start.x + dv[0];
        p_end.y = p_start.y + dv[1];
        p_end.z = p_start.z + dv[2];

        viewer.addArrow(p_end, p_start, r, g, b, false, name);
    }

    void draw_reference_frame(pcl::visualization::PCLVisualizer &viewer, Eigen::Vector3f center, pcl::ReferenceFrame rf, float size, std::string name) {

        Eigen::Vector3f ax;
        Eigen::Vector3f ay;
        Eigen::Vector3f az;

        ax << rf.x_axis[0], rf.x_axis[1], rf.x_axis[2];
        ay << rf.y_axis[0], rf.y_axis[1], rf.y_axis[2];
        az << rf.z_axis[0], rf.z_axis[1], rf.z_axis[2];
        ax = ax * size + center;
        ay = ay * size + center;
        az = az * size + center;

        std::stringstream ss;

        ss << name << "_x";
        draw_3D_vector(viewer, center, ax, 1, 0, 0, ss.str().c_str());
        ss << name << "_y";
        draw_3D_vector(viewer, center, ay, 0, 1, 0, ss.str().c_str());
        ss << name << "_z";
        draw_3D_vector(viewer, center, az, 0, 0, 1, ss.str().c_str());

    }

    void
    clusterize(pcl::PointCloud<PointType>::Ptr& cloud, std::vector<pcl::PointIndices>& cluster_indices, float cluster_tolerance, float min_cluster_size, float max_cluster_size) {
        pcl::search::KdTree<PointType>::Ptr tree2(new pcl::search::KdTree<PointType>);
        tree2->setInputCloud(cloud);

        pcl::EuclideanClusterExtraction<PointType> ec;
        ec.setClusterTolerance(cluster_tolerance);
        ec.setMinClusterSize(min_cluster_size);
        ec.setMaxClusterSize(max_cluster_size);
        ec.setSearchMethod(tree2);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

    }

    void
    compute_centroid_local_rf(pcl::PointCloud<PointType>::Ptr& cloud, pcl::ReferenceFrame& rf, int type) {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud, centroid);
        pcl::PointCloud<PointType>::Ptr centroid_cloud(new pcl::PointCloud<PointType>());
        PointType p_centroid;
        p_centroid.x = centroid(0);
        p_centroid.y = centroid(1);
        p_centroid.z = centroid(2);
        centroid_cloud->points.push_back(p_centroid);

        pcl::PointCloud<pcl::ReferenceFrame>::Ptr rf_cloud(new pcl::PointCloud<pcl::ReferenceFrame>());

        pcl::SHOTLocalReferenceFrameEstimation<PointType, pcl::ReferenceFrame> est;
        est.setInputCloud(centroid_cloud);
        est.setSearchSurface(cloud);
        est.setRadiusSearch(20000.0f);
        est.compute(*rf_cloud);

        rf = rf_cloud->points[0];
    }

    void
    compute_centroid_local_rf(pcl::PointCloud<PointType>::Ptr& cloud, pcl::ReferenceFrame& rf, Eigen::Vector3f& gravity, int type) {
        compute_centroid_local_rf(cloud, rf, type);
        Eigen::Matrix4f erf;
        convert_rf_to_eigen_4x4(rf, erf);
        Eigen::Vector3f ax;
        Eigen::Vector3f ay;
        Eigen::Vector3f az;
        ax << erf(0, 0), erf(1, 0), erf(2, 0);
        ay << erf(0, 1), erf(1, 1), erf(2, 1);
        az << erf(0, 2), erf(1, 2), erf(2, 2);
        float m_ax = fabs(ax.dot(gravity));
        float m_ay = fabs(ay.dot(gravity));
        float m_az = fabs(az.dot(gravity));

        Eigen::Matrix4f rot;
        float sign = 1.0f;
        if (m_ay > m_ax && m_ay > m_az) {
            // Y
            sign = ay.dot(gravity) > 0 ? -1 : 1;
            lar_tools::rotation_matrix_4x4('x', sign * M_PI / 2.0f, rot);
        } else if (m_az > m_ax && m_az > m_ay) {
            // Z
            sign = az.dot(gravity) > 0 ? 1 : -1;
            rot << 1, 0, 0, 0, 0, sign, 0, 0, 0, 0, sign, 0, 0, 0, 0, 1;
            //            lar_tools::rotation_matrix_4x4('x', 0, rot);
        } else {
            // X
            sign = ax.dot(gravity) > 0 ? 1 : -1;
            lar_tools::rotation_matrix_4x4('y', sign * M_PI / 2.0f, rot);
        }
        erf = erf*rot;
        convert_rf_to_eigen_4x4(rf, erf, true);
    }

    void
    convert_rf_to_eigen_4x4(pcl::ReferenceFrame& rf, Eigen::Matrix4f& vector, bool reverse) {
        if (!reverse) {

            vector <<
                    rf.x_axis[0], rf.y_axis[0], rf.z_axis[0], 0,
                    rf.x_axis[1], rf.y_axis[1], rf.z_axis[1], 0,
                    rf.x_axis[2], rf.y_axis[2], rf.z_axis[2], 0,
                    0, 0, 0, 1;

        } else {
            rf.x_axis[0] = vector(0, 0);
            rf.x_axis[1] = vector(1, 0);
            rf.x_axis[2] = vector(2, 0);

            rf.y_axis[0] = vector(0, 1);
            rf.y_axis[1] = vector(1, 1);
            rf.y_axis[2] = vector(2, 1);

            rf.z_axis[0] = vector(0, 2);
            rf.z_axis[1] = vector(1, 2);
            rf.z_axis[2] = vector(2, 2);

        }
    }

}