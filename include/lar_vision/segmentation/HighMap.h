/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * File:   HighMap.h
 * Author: daniele
 *
 * Created on November 9, 2015, 12:04 PM
 */

#ifndef HIGHMAP_H
#define HIGHMAP_H

#include <cstdlib>
#include <iostream>
#include <vector>
#include <string>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "lar_vision_commons.h"

namespace lar_vision {

    class HighMap {
    public:
        HighMap(double max_size, double step, double offset, double reduction = 1.0);
        HighMap(const HighMap& orig);
        virtual ~HighMap();

        void clear();
        void pinPoint(double z);
        int pointIndex(double z);
        double pointValue(double z);
        void planesCheck(
                pcl::PointCloud<PointType>::Ptr& cloud,
                pcl::PointCloud<NormalType>::Ptr& cloud_normals,
                std::vector<int>& filtered_indices,
                std::vector<int>& planes_indices,
                float max_angle,
                int map_min_inliers
                );

        double highest_plane_z;
    private:
        double* map;
        double size;
        double max_size;
        double step;
        double offset;
        double reduction;
    };
}

#endif /* HIGHMAP_H */
