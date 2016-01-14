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

#include "HighMap.h"
namespace lar_vision {

HighMap::HighMap(double max_size, double step, double offset, double reduction) {
        this->size = max_size / step;
        this->map = new double[(int) size];
        this->max_size = max_size;
        this->step = step;
        this->offset = offset;
        this->reduction = reduction;
        this->max_z = -std::numeric_limits<double>::max();
        this->min_z = std::numeric_limits<double>::max();
        std::fill(this->map, this->map + (int) this->size, 0.0);
}

HighMap::HighMap(const HighMap& orig) {

}

HighMap::~HighMap() {

}

void HighMap::clear() {
        std::fill(this->map, this->map + (int) this->size, 0.0);
}

void HighMap::pinPoint(double z) {
        int iz = floor(z / step);
        iz += floor(this->offset / step);
        if (iz<this->size && iz >= 0) {
                this->min_z = z < this->min_z ? z : this->min_z;
                this->max_z = z > this->max_z ? z : this->max_z;
                this->map[iz]++;
        }
}

int HighMap::pointIndex(double z) {
        int iz = floor(z / step);
        iz += floor(this->offset / step);
        if (iz<this->size && iz >= 0) {
                return iz;
        }
        return -1;
}

double HighMap::pointValue(double z) {
        int iz = floor(z / step);
        iz += floor(this->offset / step);
        if (iz<this->size && iz >= 0) {

                return this->map[iz];
        }
        return -1;
}

void HighMap::planesCheck(
        pcl::PointCloud<PointType>::Ptr& cloud,
        pcl::PointCloud<NormalType>::Ptr& cloud_normals,
        std::vector<int>& filtered_indices,
        std::vector<int>& planes_indices,
        float max_angle,
        int map_min_inliers) {


        this->highest_plane_z = -std::numeric_limits<double>::max();

        Eigen::Vector3f normal;
        Eigen::Vector3f gravity_neg(0, 0, 1);


        for (int i = 0; i < cloud_normals->points.size(); i += reduction) {
                NormalType n = cloud_normals->points[i];
                PointType p = cloud->points[i];
                normal(0) = n.normal_x;
                normal(1) = n.normal_y;
                normal(2) = n.normal_z;

                float angle = acos(normal.dot(gravity_neg));
                if (angle <= max_angle * M_PI / 180.0f) {
                        this->pinPoint(p.z);
                }
        }

        /*for(int i = 0; i < this->size; i++){
            std::cout << "Map "<<i<<" "<<this->map[i]<<std::endl;
        }*/

        float max_angle_rad = max_angle * M_PI / 180.0f;
        for (int i = 0; i < cloud->points.size(); i++) {
                PointType p = cloud->points[i];
                NormalType n = cloud_normals->points[i];
                normal(0) = n.normal_x;
                normal(1) = n.normal_y;
                normal(2) = n.normal_z;

                float angle = acos(normal.dot(gravity_neg));

                if (this->pointValue(p.z) >= map_min_inliers && angle <= max_angle_rad) {

                        planes_indices.push_back(i);
                        this->highest_plane_z = p.z>this->highest_plane_z ? p.z : this->highest_plane_z;
                } //            else if (this->pointValue(p.z - step / 2.0f) >= map_min_inliers) {
                  //                planes_indices.push_back(i);
                  //            }
                else if (this->pointValue(p.z + step / 2.0f) >= map_min_inliers  && angle <= max_angle_rad) {
                        planes_indices.push_back(i);
                } else {
                        filtered_indices.push_back(i);
                }

        }


}



}
