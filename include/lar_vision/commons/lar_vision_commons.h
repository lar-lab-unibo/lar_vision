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

#ifndef LAR_VISION_COMMONS_H
#define LAR_VISION_COMMONS_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <pcl/visualization/pcl_visualizer.h>


namespace lar_vision {

    typedef pcl::PointXYZRGB PointType;
    typedef pcl::Normal NormalType;

    /**
     * Palette of colors
     */
    struct Palette {
        std::vector<Eigen::Vector3i> colors;
        int index;

        Palette() {
            index = -1;
            colors.push_back(Eigen::Vector3i(115, 255, 0));
            colors.push_back(Eigen::Vector3i(232, 159, 12));
            colors.push_back(Eigen::Vector3i(255, 0, 0));
            colors.push_back(Eigen::Vector3i(61, 12, 232));
            colors.push_back(Eigen::Vector3i(13, 255, 239));
        }

        Eigen::Vector3i getColor() {
            index++;
            index = index % colors.size();
            return colors[index];
        }
    };


    /**
     * Displays Cloud in viewer
     * @param viewer target viewer
     * @param cloud target cloud
     * @param r Red
     * @param g Green
     * @param b Blue
     * @param size point size
     * @param name ID_NAME for viewer (no duplicates)
     */
    void
    display_cloud(pcl::visualization::PCLVisualizer &viewer, pcl::PointCloud<PointType>::Ptr& cloud, int r, int g, int b, int size, std::string name);

    /**
     * Compute Normals on an unorganized cloud
     */
    void
    compute_normals(pcl::PointCloud<PointType>::Ptr& cloud, pcl::PointCloud<NormalType>::Ptr& cloud_normals, float radius_search = 0.03f);

    /**
     * Clusterize Point Cloud
     */
    void
    clusterize(pcl::PointCloud<PointType>::Ptr& cloud, std::vector<pcl::PointIndices>& cluster_indices, float cluster_tolerance = 0.01f, float min_cluster_size = 100, float max_cluster_size = 250000);

    /*
     * Conversion between PCL POint and Eigen Vector
     */
    void
    convert_point_3D(PointType& pt, Eigen::Vector3f& p, bool reverse);

    /**
     */
    void
    draw_3D_vector(pcl::visualization::PCLVisualizer& viewer, Eigen::Vector3f start, Eigen::Vector3f end, float r, float g, float b, std::string name);

    /**
     */
    void
    draw_text_3D(pcl::visualization::PCLVisualizer &viewer, std::string text, Eigen::Vector3f center, float r, float g, float b, float size, std::string name);

    /**
     */
    void
    draw_reference_frame(pcl::visualization::PCLVisualizer &viewer, Eigen::Vector3f center, pcl::ReferenceFrame rf, float size, std::string name);

    /**
     */
    void
    draw_reference_frame(pcl::visualization::PCLVisualizer &viewer, Eigen::Matrix4d& rf, float size, std::string name);

    /**
     */
    void
    compute_centroid_local_rf(pcl::PointCloud<PointType>::Ptr& cloud, pcl::ReferenceFrame& rf, int type = -1);

    /**
     */
    void
    compute_centroid_local_rf(pcl::PointCloud<PointType>::Ptr& cloud, pcl::ReferenceFrame& rf, Eigen::Vector3f& gravity, int type = -1);

    /**
     *
     * @param rf
     * @param vector
     */
    void
    convert_rf_to_eigen_4x4(pcl::ReferenceFrame& rf, Eigen::Matrix4f& vector, bool reverse = false);
}

#endif /* LAR_VISION_COMMONS_H */
