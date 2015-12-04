/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * File:   CrabbyGripper.cpp
 * Author: daniele
 *
 * Created on November 10, 2015, 7:16 PM
 */

#include "CrabbyGripper.h"

namespace lar_vision {

CrabbyGripper::CrabbyGripper(int type, double min_offset,double max_offset, double min, double max, double epsilon,double fritction_cone_angle, double ortogonal_range, double max_curvature) {
        this->type = type;
        this->min_offset = min_offset;
        this->max_offset = max_offset;
        this->min_radius = min;
        this->max_radius = max;
        this->max_curvature = max_curvature;
        this->epsilon = epsilon;
        this->fritction_cone_angle = fritction_cone_angle;
        this->ortogonal_range = ortogonal_range;
        this->discard_vertices = true;
        this->auto_discard_planar_invalids = false;
}

CrabbyGripper::~CrabbyGripper() {
}

void CrabbyGripper::find(std::vector<GrasperPoint>& points, std::vector<int>& indices, int jump) {
        Eigen::Vector2f target;
        Eigen::Vector2f pointer;
        double offset_distance;
        double offset_orientation;
        indices.clear();
        int found = 0;
        for (int i = 0; i < points.size(); i++) {
                if(points[i].curvature > this->max_curvature) continue;
                target = points[i].p;

                for (int j = 0; j < points.size(); j++) {
                        if (i != j) {
                                if(points[j].curvature > this->max_curvature) continue;

                                pointer = points[j].p;
                                offset_distance = (target - pointer).norm();

                                //if (fabs(offset_distance - this->offset) < this->epsilon) {
                                if (offset_distance>= this->min_offset && offset_distance <= this->max_offset) {
                                        Eigen::Vector2f middle = (target + pointer)*0.5f;
                                        Eigen::Vector2f len = (target - pointer)*(1.0f / (target - pointer).norm());


                                        for (int k = 0; k < points.size(); k++) {
                                                if (k != j && k != i) {
                                                        if(points[k].curvature > this->max_curvature) continue;

                                                        Eigen::Vector2f third = points[k].p;
                                                        Eigen::Vector2f dir = (third - middle)*(1.0f / (third - middle).norm());

                                                        offset_orientation = acos(dir.dot(len));
                                                        //                                    std::cout << "orientation: "<<offset_orientation<<std::endl;
                                                        if (offset_orientation >= (M_PI / 2.0f) - this->ortogonal_range && offset_orientation <= (M_PI / 2.0f) + this->ortogonal_range) {
                                                                offset_distance = (third - middle).norm() - this->epsilon * 2;
                                                                if (offset_distance>this->min_radius && offset_distance<this->max_radius) {
                                                                        if(this->auto_discard_planar_invalids) {
                                                                                indices.push_back(i);
                                                                                indices.push_back(j);
                                                                                indices.push_back(k);
                                                                                if(this->isValidPlanarConfiguration(points,indices)) {
                                                                                        jump--;
                                                                                        if(jump<=0) {
                                                                                                return;
                                                                                        }else{
                                                                                                indices.clear();
                                                                                        }
                                                                                }else{
                                                                                        indices.clear();
                                                                                }
                                                                        }else{
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
}


bool CrabbyGripper::isValidPlanarConfiguration(std::vector<GrasperPoint>& points,std::vector<int>& indices) {
        std::vector<GrasperPoint> filtered_points;

        for (int i = 0; i < indices.size(); i++) {
                filtered_points.push_back(points[indices[i]]);
        }
        return isValidPlanarConfiguration(filtered_points);
}

bool CrabbyGripper::isValidPlanarConfiguration(std::vector<GrasperPoint>& points) {

        if(points.size() >= 2) {
                Eigen::Vector2f p1 = points[0].p;
                Eigen::Vector2f p2 = points[1].p;
                Eigen::Vector2f n1 = points[0].normal;
                Eigen::Vector2f n2 = points[1].normal;
                Eigen::Vector2f d1 = (p2-p1)*(1.0f / (p2-p1).norm());
                Eigen::Vector2f d2 = (p1-p2)*(1.0f / (p1-p2).norm());

                double a1 = acos(n1.dot(d1));
                double a2 = acos(n2.dot(d2));
                double angle_limit = (M_PI*2.0-fritction_cone_angle)/2.0;

                if(a1>angle_limit && a2>angle_limit) {
                        return true;
                }
        }
        return false;
}

bool CrabbyGripper::isVectorPositiveCombinationOf(Eigen::Vector2f& vector, std::vector<GrasperPoint>& points) {

        return false;
}

bool CrabbyGripper::getApproachRF(std::vector<GrasperPoint>& points,std::vector<int>& configuration_indices,double& x,double& y,double &z,double& roll,double& pitch,double& yaw){
        std::vector<GrasperPoint> configuration_points;
        for (int i = 0; i < configuration_indices.size(); i++)
                configuration_points.push_back(points[configuration_indices[i]]);

        if(configuration_points.size()<3) return false;



        Eigen::Vector2f middle = (configuration_points[0].p + configuration_points[1].p)*0.5f;
        Eigen::Vector2f dir =  (configuration_points[2].p - middle);

        double apporach_distance = dir.norm();
        Eigen::Vector2f len = (configuration_points[0].p - configuration_points[1].p)*(1.0f / (configuration_points[0].p - configuration_points[1].p).norm());
        Eigen::Vector2f L(len[1],-len[0]);
        Eigen::Vector2f L_meter=L*apporach_distance;
        Eigen::Vector2f check1 = middle+L_meter;
        Eigen::Vector2f check2 = middle-L_meter;
        Eigen::Vector2f check;
        if(acos(dir.dot(check1))>acos(dir.dot(check2))) {
                check=check2;
        }else{
                check=check1;
                L = -L;
        }

        Eigen::Vector2f check_unit = check*(1.0/check.norm());
        x = check[0];
        y = check[1];
        z = 0.0;
        roll = 0;
        pitch = 0;
        yaw =  atan2(L[1],L[0]);

        return true;
}

}
