/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * File:   CrabbyGripper.h
 * Author: daniele
 *
 * Created on November 10, 2015, 7:16 PM
 */

#ifndef CRABBYGRIPPER_H
#define CRABBYGRIPPER_H

#include "Grasper.h"

#define CRABBY_GRIPPER_STATUS_PARALLEL 0
#define CRABBY_GRIPPER_STATUS_TRIPOD 1
#define CRABBY_GRIPPER_STATUS_DUAL 2
#define CRABBY_GRIPPER_EPSILON 0.01
#define CRABBY_GRIPPER_ORTHOGONAL_RANGE 0.087222222
#define CRABBY_GRIPPER_FRICTION_CONE_ANGLE 0.523333333
#define CRABBY_GRIPPER_MAX_CURVATURE 1.0

namespace lar_vision {

    class CrabbyGripper {
    public:
        CrabbyGripper(int type = CRABBY_GRIPPER_STATUS_PARALLEL, double min_offset = 0.001f,double max_offset = 0.2f, double min = 0.01f, double max = 0.05f, double epsilon = CRABBY_GRIPPER_EPSILON, double fritction_cone_angle = CRABBY_GRIPPER_FRICTION_CONE_ANGLE, double ortogonal_range = CRABBY_GRIPPER_ORTHOGONAL_RANGE, double max_curvature = CRABBY_GRIPPER_MAX_CURVATURE);
        virtual ~CrabbyGripper();
        void find(std::vector<GrasperPoint>& points, std::vector<int>& indices, int jump = 0);
        bool isValidPlanarConfiguration(std::vector<GrasperPoint>& points,std::vector<int>& indices);
        bool isValidPlanarConfiguration(std::vector<GrasperPoint>& points);
        bool isVectorPositiveCombinationOf(Eigen::Vector2f& vector, std::vector<GrasperPoint>& points);

        bool getApproachRF(std::vector<GrasperPoint>& points,std::vector<int>& configuration_indices,double& x,double& y,double &z,double& roll,double& pitch,double& yaw);

        double max_offset;
        double min_offset;
        double min_radius;
        double max_radius;
        double max_curvature;
        double fritction_cone_angle;
        double ortogonal_range;
        double epsilon;
        bool discard_vertices;
        bool auto_discard_planar_invalids;
        int type;
    private:


    };
}
#endif /* CRABBYGRIPPER_H */
