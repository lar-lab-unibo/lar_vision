/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * File:   GraspingGripper.h
 * Author: daniele
 *
 * Created on November 10, 2015, 7:16 PM
 */

#ifndef GRIPPER_H
#define GRIPPER_H

#include "Grasper.h"

#define GRASPING_GRIPPER_STATUS_PARALLEL 0
#define GRASPING_GRIPPER_STATUS_TRIPOD 1
#define GRASPING_GRIPPER_STATUS_DUAL 2
#define GRASPING_GRIPPER_EPSILON 0.01
#define GRASPING_GRIPPER_ORTHOGONAL_RANGE 0.087222222
#define GRASPING_GRIPPER_MAX_CURVATURE 1.0

namespace lar_vision {

    class Gripper {
    public:
        virtual void find(std::vector<GrasperPoint>& points, std::vector<int>& indices, int jump = 0) = 0;

        double offset;
        double min_radius;
        double max_radius;
        double max_curvature;
        double ortogonal_range;
        double epsilon;
        bool discard_vertices;
        int type;
    private:


    };
}
#endif /* GRIPPER_H */
