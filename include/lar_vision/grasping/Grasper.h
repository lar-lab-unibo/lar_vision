/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * File:   Grasper.h
 * Author: daniele
 *
 * Created on November 10, 2015, 7:05 PM
 */

#ifndef GRASPER_H
#define GRASPER_H

#include "lar_vision_commons.h"
#include <pcl/common/io.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>

#define LAR_VISION_GRASPER_POINT_TYPE_CONTACT_POINT 100
#define LAR_VISION_GRASPER_POINT_TYPE_CONTROL_POINT 105

#define LAR_VISION_GRASPER_HULL_TYPE_CONVEX 501
#define LAR_VISION_GRASPER_HULL_TYPE_CONCAVE 502

namespace lar_vision {

    /**
       Grasper POINT Representation
     */
    struct GrasperPoint {
        Eigen::Vector2f p;
        Eigen::Vector2f normal;
        double curvature;
        GrasperPoint* next;
        GrasperPoint* back;
        int point_type;
        bool vertex;

        GrasperPoint(double x, double y) {
            this->p << x, y;
            this->vertex = false;
            this->next = NULL;
            this->back = NULL;
            this->point_type = LAR_VISION_GRASPER_POINT_TYPE_CONTACT_POINT;
        }

        GrasperPoint(const Eigen::Vector2f& p) {
            this->p << p(0), p(1);
            this->vertex = false;
            this->next = NULL;
            this->back = NULL;
            this->point_type = LAR_VISION_GRASPER_POINT_TYPE_CONTACT_POINT;
        }
    };

    /**
     * Grasper LINE Representation
     */
    struct GrasperLine {
        Eigen::Vector2f p1;
        Eigen::Vector2f p2;
        double norm;
        std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > points;

        GrasperLine(double x1, double y1, double x2, double y2, double step) {
            init(x1, y1, x2, y2, step);
        }

        GrasperLine(GrasperPoint p1, GrasperPoint p2, double step) {
            init(p1.p(0), p1.p(1), p2.p(0), p2.p(1), step);
        }

        void init(double x1, double y1, double x2, double y2, double step) {
            this->p1 << x1, y1;
            this->p2 << x2, y2;
            this->norm = (this->p1 - this->p2).norm();

            Eigen::Vector2f dir;
            dir = (this->p2 - this->p1)*(1 / this->norm);
            dir = dir * step;

            this->points.push_back(this->p1);
            double d_norm = dir.norm();
            Eigen::Vector2f current = this->p1;
            while (d_norm < this->norm) {
                current = current + dir;
                points.push_back(current);
                d_norm += dir.norm();
            }
            this->points.push_back(this->p2);
        }

        Eigen::Vector2f ortogonal() {
            Eigen::Vector2f dir = (this->p2 - this->p1)*(1.0f / this->norm);
            Eigen::Vector2f orto;
            orto << -dir(1), dir(0);
            return orto;
        }

    };

    class Grasper {
    public:
        Grasper(double concave_alpha = 0.1f, double discretization_step = 0.01f);
        virtual ~Grasper();
        void setCloud(pcl::PointCloud<PointType>::Ptr& cloud);

        std::vector<GrasperLine> lines;
        std::vector<GrasperPoint> points;
        std::vector<GrasperPoint> normals;
        Eigen::Vector2f centroid;
        int debug_jump;
        pcl::PointCloud<PointType>::Ptr hull;
        int hull_type;
    private:
        void refinePoints();
        void computeNormals();
        void computeCentroid();

        double concave_alpha;
        double discretization_step;

        pcl::PointCloud<PointType>::Ptr cloud;


        int gripper_status;


    };
}
#endif /* GRASPER_H */
