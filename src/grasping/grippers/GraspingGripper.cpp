/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * File:   GraspingGripper.cpp
 * Author: daniele
 *
 * Created on November 10, 2015, 7:16 PM
 */

#include "GraspingGripper.h"

namespace lar_vision {

    GraspingGripper::GraspingGripper(int type, double offset, double min, double max, double epsilon, double ortogonal_range, double max_curvature) {
        this->type = type;
        this->offset = offset;
        this->min_radius = min;
        this->max_radius = max;
        this->max_curvature = max_curvature;
        this->epsilon = epsilon;
        this->ortogonal_range = ortogonal_range;
        this->discard_vertices = true;
    }

    GraspingGripper::~GraspingGripper() {
    }

    void GraspingGripper::find(std::vector<GrasperPoint>& points, std::vector<int>& indices, int jump) {
        if (this->type == GRASPING_GRIPPER_STATUS_PARALLEL) {
            findParallel(points, indices, jump);
        }
    }

    void GraspingGripper::findParallel(std::vector<GrasperPoint>& points, std::vector<int>& indices, int jump) {
        Eigen::Vector2f target;
        Eigen::Vector2f pointer;
        double offset_distance;
        double offset_orientation;
        indices.clear();
        int found = 0;
        for (int i = 0; i < points.size(); i++) {
            if(points[i].curvature > this->max_curvature)continue;
            target = points[i].p;

            for (int j = 0; j < points.size(); j++) {
                if (i != j) {
                    if(points[j].curvature > this->max_curvature)continue;

                    pointer = points[j].p;
                    offset_distance = (target - pointer).norm();

                    if (fabs(offset_distance - this->offset) < this->epsilon) {
                        Eigen::Vector2f middle = (target + pointer)*0.5f;
                        Eigen::Vector2f len = (target - pointer)*(1.0f / (target - pointer).norm());


                        for (int k = 0; k < points.size(); k++) {
                            if (k != j && k != i) {
                                if(points[k].curvature > this->max_curvature)continue;

                                Eigen::Vector2f third = points[k].p;
                                Eigen::Vector2f dir = (third - middle)*(1.0f / (third - middle).norm());

                                offset_orientation = acos(dir.dot(len));
                                //                                    std::cout << "orientation: "<<offset_orientation<<std::endl;
                                if (offset_orientation >= (M_PI / 2.0f) - this->ortogonal_range && offset_orientation <= (M_PI / 2.0f) + this->ortogonal_range) {
                                    offset_distance = (third - middle).norm() - this->epsilon * 2;
                                    if (offset_distance>this->min_radius && offset_distance<this->max_radius) {
                                        jump--;
                                        if (jump <= 0) {
                                            indices.push_back(i);
                                            indices.push_back(j);
                                            indices.push_back(k);
                                            return;
                                        }
                                    }
                                }
                            }
                        }

                    }
                }
            }
        }
    }

    bool GraspingGripper::isValidPlanarConfiguration(std::vector<GrasperPoint>& points,std::vector<int>& indices) {
        std::vector<GrasperPoint> filtered_points;

        for (int i = 0; i < indices.size(); i++) {
            filtered_points.push_back(points[indices[i]]);
        }
        return isValidPlanarConfiguration(points);
    }

    bool GraspingGripper::isValidPlanarConfiguration(std::vector<GrasperPoint>& points) {
      
        Eigen::Vector2f north;
        north << 0, -1;
        Eigen::Matrix2f rot;
        bool valid = true;
        for (float angle = 0; angle < 2 * M_PI; angle += M_PI / 4.0f) {
            rot << cos(angle), -sin(angle), sin(angle), cos(angle);
            north = rot*north;
            valid = valid && isVectorPositiveCombinationOf(north, points);
        }
        return valid;
    }

    bool GraspingGripper::isVectorPositiveCombinationOf(Eigen::Vector2f& vector, std::vector<GrasperPoint>& points) {

        for (int i = 0; i < points.size(); i++) {
            if (vector.dot(points[i].normal) > 0)return true;
        }
        return false;
    }
}
