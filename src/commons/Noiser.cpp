/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * File:   Noiser.cpp
 * Author: daniele
 *
 * Created on November 18, 2015, 6:56 PM
 */

#include "Noiser.h"

namespace lar_vision {

    Noiser::Noiser(float axial_coefficient, float axial_bias, float axial_offset, float lateral_coefficient, float lateral_default_focal) {
        this->axial_coefficient = axial_coefficient;
        this->axial_bias = axial_bias;
        this->axial_offset = axial_offset;
        this->lateral_coefficient = lateral_coefficient;
        this->lateral_default_focal = lateral_default_focal;
    }

    Noiser::~Noiser() {

    }

    void Noiser::setCloud(pcl::PointCloud<PointType>::Ptr& cloud) {
        this->cloud = cloud;
    }

    float Noiser::randomFloat(float min, float max) {
        float random = ((float) rand()) / (float) RAND_MAX;
        float range = max - min;
        return (random * range) +min;
    }

    void Noiser::addNoise(pcl::PointCloud<PointType>::Ptr& cloud_out) {

        cloud_out->points.clear();
        cloud_out->width = cloud->width;
        cloud_out->height = cloud->height;

        pcl::copyPointCloud(*cloud,*cloud_out);
        for (int i = 0; i < cloud->points.size(); i++) {
            float z = cloud_out->points[i].z;
            cloud_out->points[i].z += randomFloat(-zNoise(z),zNoise(z));
            cloud_out->points[i].x += randomFloat(-xyNoise(z),xyNoise(z));
            cloud_out->points[i].y += randomFloat(-xyNoise(z),xyNoise(z));
        }

    }

    float Noiser::zNoise(float z_in) {
        return this->axial_bias + this->axial_coefficient * (z_in - this->axial_offset)*(z_in - this->axial_offset);
    }

    float Noiser::xyNoise(float z_in) {
        return z_in * this->lateral_coefficient / this->lateral_default_focal;
    }


}
