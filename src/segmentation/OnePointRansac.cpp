/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * File:   HighMap.cpp
 * Author: daniele
 *
 * Created on November 9, 2015, 12:04 PM
 */

#include "OnePointRansac.h"
#include <stdlib.h>
#include <pcl/filters/extract_indices.h>

namespace lar_vision {

OnePointRansac::OnePointRansac(double distance_th, double probability, int max_iterations, double min_cosangle_th) {
        this->distance_th = distance_th;
        this->probability = probability;
        this->max_iterations = max_iterations;
        this->min_cosangle_th = min_cosangle_th;
}

OnePointRansac::OnePointRansac(const OnePointRansac& orig) {

}

OnePointRansac::~OnePointRansac() {

}

void OnePointRansac::planesCheck(
        pcl::PointCloud<PointType>::Ptr& cloud,
        pcl::PointCloud<NormalType>::Ptr& cloud_normals,
        std::vector<int>& filtered_indices,
        std::vector<int>& planes_indices,
        float max_angle,
        int map_min_inliers
        ){

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients model_coefficients;
        inliers->indices.clear ();
        model_coefficients.values.clear ();


        inliers->header = model_coefficients.header = cloud->header;
        model_coefficients.values.resize (4);

        size_t best_score = 0;
        const size_t no_of_pts = cloud->points.size();
        const double no_of_pts_inverse = 1.0/(double)no_of_pts;
        const double lognum = std::log(1-this->probability);

        //Random
        srand (time(NULL));
        unsigned current_max_iterations = this->max_iterations;


        unsigned it = 0;
        for(; it < current_max_iterations; ++it)
        {
                const int idx = rand() % no_of_pts;
                const Eigen::Vector3f& current_plane_normal = cloud_normals->points[idx].getNormalVector3fMap ();
                float current_plane_offset = current_plane_normal.dot(cloud->points[idx].getVector3fMap ());

                size_t current_score = 0;
                for (size_t k = 0; k < no_of_pts; ++k)
                {
                        const int kidx = k;
                        if (current_plane_normal.dot (cloud_normals->points[kidx].getNormalVector3fMap ()) > this->min_cosangle_th &&
                            (((current_plane_offset - current_plane_normal.dot (cloud->points[kidx].getVector3fMap ())) /
                              current_plane_normal.dot (cloud_normals->points[kidx].getNormalVector3fMap ())) *
                             cloud_normals->points[kidx].getNormalVector3fMap ()).squaredNorm () < this->distance_th)
                                ++current_score;
                }

                // update
                if(current_score > best_score)
                {
                        best_score = current_score;
                        model_coefficients.values[0] = current_plane_normal.x ();
                        model_coefficients.values[1] = current_plane_normal.y ();
                        model_coefficients.values[2] = current_plane_normal.z ();
                        model_coefficients.values[3] = current_plane_offset;

                        if(best_score == no_of_pts)
                                current_max_iterations = 0;
                        else
                                current_max_iterations = std::min((double)this->max_iterations, std::ceil(lognum / std::log(1.0 - std::pow((double)best_score * no_of_pts_inverse, 3.0))));
                }
        }

        Eigen::Map<Eigen::Vector3f> plane_normal (&(model_coefficients.values[0]));
        for (size_t k = 0; k < no_of_pts; ++k)
        {
                const int kidx = k;
                if (plane_normal.dot (cloud_normals->points[kidx].getNormalVector3fMap ()) > this->min_cosangle_th &&
                    (((model_coefficients.values[3] - plane_normal.dot (cloud->points[kidx].getVector3fMap ())) /
                      plane_normal.dot (cloud_normals->points[kidx].getNormalVector3fMap ())) *
                     cloud_normals->points[kidx].getNormalVector3fMap ()).squaredNorm () < this->distance_th)
                        inliers->indices.push_back (kidx);
        }


        pcl::ExtractIndices<PointType> extract;
       extract.setInputCloud (cloud);
       extract.setIndices (inliers);
       extract.setNegative (false);

       extract.filter (planes_indices);

       extract.setNegative (true);
       extract.filter (filtered_indices);
}


}
