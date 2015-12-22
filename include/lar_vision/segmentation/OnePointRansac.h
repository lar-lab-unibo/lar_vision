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

#ifndef ONEPOINTRANSAC_H
#define ONEPOINTRANSAC_H

#include <cstdlib>
#include <iostream>
#include <vector>
#include <string>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "lar_vision_commons.h"

namespace lar_vision {

    class OnePointRansac {
    public:
        OnePointRansac(double distance_th = 0.01, double probability = 0.99, int max_iterations = 1000, double min_cosangle_th = std::cos(0.707));
        OnePointRansac(const OnePointRansac& orig);
        virtual ~OnePointRansac();

        void planesCheck(
          pcl::PointCloud<PointType>::Ptr& cloud,
          pcl::PointCloud<NormalType>::Ptr& cloud_normals,
          std::vector<int>& filtered_indices,
          std::vector<int>& planes_indices,
          float max_angle,
          int map_min_inliers
        );


    private:
        double distance_th;
        double probability;
        int max_iterations;
        double min_cosangle_th;

    };
}

#endif /* ONEPOINTRANSAC_H */
