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

#ifndef GRASPINGGRIPPER_H
#define GRASPINGGRIPPER_H

#include "Grasper.h"

#define GRASPING_GRIPPER_STATUS_PARALLEL 0
#define GRASPING_GRIPPER_STATUS_TRIPOD 1
#define GRASPING_GRIPPER_STATUS_DUAL 2
#define GRASPING_GRIPPER_EPSILON 0.0001
#define GRASPING_GRIPPER_ORTHOGONAL_RANGE 0.087222222


namespace lar_vision {

    class GraspingGripper {
    public:
        GraspingGripper(int type = GRASPING_GRIPPER_STATUS_PARALLEL, double offset = 0.05f, double min = 0.005f, double max = 0.4f, double epsilon = GRASPING_GRIPPER_EPSILON, double ortogonal_range = GRASPING_GRIPPER_ORTHOGONAL_RANGE);
        virtual ~GraspingGripper();
        void find(std::vector<GrasperPoint>& points, std::vector<int>& indices, int jump = 0);
        void findParallel(std::vector<GrasperPoint>& points, std::vector<int>& indices, int jump = 0);
    private:
        double offset;
        double min_radius;
        double max_radius;
        double ortogonal_range;
        double epsilon;
        int type;
    };
}
#endif /* GRASPINGGRIPPER_H */

