/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Slicer.cpp
 * Author: daniele
 * 
 * Created on November 10, 2015, 4:16 PM
 */

#include "Slicer.h"

namespace lar_vision {

    Slicer::Slicer(float slice_size) {
        this->slice_size = slice_size;
    }

    Slicer::~Slicer() {
    }

    void Slicer::slice(pcl::PointCloud<PointType>::Ptr& cloud, std::vector<pcl::PointIndices>& slice_indices) {
        
        float min_z = 2000.0f;
        float max_z = -2000.0f;

        for (int i = 0; i < cloud->points.size(); i++) {
            PointType p = cloud->points[i];
            min_z = p.z < min_z ? p.z : min_z;
            max_z = p.z > max_z ? p.z : max_z;
        }
        
        slice_indices.clear();
        for (float z = min_z; z <= max_z; z += slice_size) {
            pcl::PointIndices indices;
            slice_indices.push_back(indices);
        }

        for (int i = 0; i < cloud->points.size(); i++) {
            PointType p = cloud->points[i];
            float index = floor((p.z - min_z) / slice_size);
            slice_indices[(int) index].indices.push_back(i);
        }
    }

}
