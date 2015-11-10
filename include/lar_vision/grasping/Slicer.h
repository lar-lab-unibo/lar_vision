/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Slicer.h
 * Author: daniele
 *
 * Created on November 10, 2015, 4:16 PM
 */

#ifndef SLICER_H
#define SLICER_H

#include <cstdlib>
#include <iostream>
#include <vector>
#include <string>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "lar_vision_commons.h"

namespace lar_vision {

    class Slicer {
    public:
        Slicer(float slice_size);
        virtual ~Slicer();
        void slice(pcl::PointCloud<PointType>::Ptr& cloud, std::vector<pcl::PointIndices>& slice_indices);
    private:
        float slice_size;
    };
}

#endif /* SLICER_H */

