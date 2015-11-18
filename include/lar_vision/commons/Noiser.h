/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Noiser.h
 * Author: daniele
 *
 * Created on November 18, 2015, 6:56 PM
 */

#ifndef NOISER_H
#define NOISER_H

#include "lar_vision_commons.h"

#define LAR_VISION_NOISER_AXIAL_COEFFICENT 0.0019
#define LAR_VISION_NOISER_AXIAL_BIAS 0.0012
#define LAR_VISION_NOISER_AXIAL_OFFSET 0.4
#define LAR_VISION_NOISER_LATERAL_COEFFICIENT 0.815
#define LAR_VISION_NOISER_LATERAL_DEFAULT_FOCAL 585

namespace lar_vision {

    class Noiser {
    public:
        Noiser(float axial_coefficient = LAR_VISION_NOISER_AXIAL_COEFFICENT,
                float axial_bias = LAR_VISION_NOISER_AXIAL_BIAS,
                float axial_offset = LAR_VISION_NOISER_AXIAL_OFFSET,
                float lateral_coefficient = LAR_VISION_NOISER_LATERAL_COEFFICIENT,
                float lateral_default_focal = LAR_VISION_NOISER_LATERAL_DEFAULT_FOCAL);

        virtual ~Noiser();

        float axial_coefficient;
        float axial_bias;
        float axial_offset;
        float lateral_coefficient;
        float lateral_default_focal;

        void setCloud(pcl::PointCloud<PointType>::Ptr& cloud);
        void addNoise(pcl::PointCloud<PointType>::Ptr& cloud_out);
        float zNoise(float z_in);
        float xyNoise(float z_in);
    private:
        pcl::PointCloud<PointType>::Ptr cloud;
        float randomFloat(float a, float b);
    };
}
#endif /* NOISER_H */

