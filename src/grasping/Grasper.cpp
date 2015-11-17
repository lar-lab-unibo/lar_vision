/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Grasper.cpp
 * Author: daniele
 * 
 * Created on November 10, 2015, 7:05 PM
 */

#include "Grasper.h"
namespace lar_vision {

    Grasper::Grasper(double concave_alpha, double discretization_step) {
        this->concave_alpha = concave_alpha;
        this->discretization_step = discretization_step;
        this->debug_jump = 0;
    }

    Grasper::~Grasper() {
    }

    void Grasper::setCloud(pcl::PointCloud<PointType>::Ptr& cloud) {
        this->cloud = cloud;
        this->hull = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
        this->lines.clear();

        pcl::ConcaveHull<PointType> chull;
        chull.setInputCloud(this->cloud);
        chull.setAlpha(this->concave_alpha);
        chull.reconstruct(*this->hull);

        //LINES
        PointType last;
        PointType first;
        for (int i = 0; i < this->hull->points.size(); i++) {
            PointType current;
            current = this->hull->points[i];
            if (i > 0) {
                GrasperLine line(last.x, last.y, current.x, current.y, this->discretization_step);
                this->lines.push_back(line);
            } else {
                first = current;
            }
            last = current;
        }
        GrasperLine line(first.x, first.y, last.x, last.y, this->discretization_step);
        this->lines.push_back(line);

        //REFINE POINTS
        this->refinePoints();
        this->computeCentroid();
        this->computeNormals();
    }

    void Grasper::refinePoints() {
        this->points.clear();
        for (int i = 0; i < this->lines.size(); i++) {
            for (int ip = 0; ip < this->lines[i].points.size(); ip++) {
                if (ip > 0 || i == 0) {
                    GrasperPoint p(lines[i].points[ip](0), lines[i].points[ip](1));
                    if (ip == this->lines[i].points.size() - 1) {
                        p.vertex = true;
                    }
                    this->points.push_back(p);
                }
            }
        }

        for (int i = 0; i < this->points.size(); i++) {
            if (i > 0) {
                this->points[i].back = &this->points[i - 1];
                this->points[i - 1].next = &this->points[i];
            }
        }
        if (this->points.size() > 0) {
            this->points[0].back = &this->points[this->points.size() - 1];
            this->points[this->points.size() - 1].next = &this->points[0];
        }
    }

    void Grasper::computeCentroid() {
        this->centroid << 0, 0;
        for (int i = 0; i < points.size(); i++) {
            this->centroid = this->centroid + points[i].p;
        }
        this->centroid = this->centroid * (1.0f / points.size());
    }

    void Grasper::computeNormals() {
        this->normals.clear();

        int depth;
        Eigen::Vector2f dir;
        dir << 0, 0;
        if (points.size() < 3)return;
        for (int i = 0; i < points.size(); i++) {
            GrasperPoint current = points[i];
            GrasperPoint* next = current.next;
            GrasperPoint* back = current.back;
            depth = 2;
            while (next != NULL && back != NULL && depth >= 0) {
                back = back->back;
                next = next->next;
                depth--;
            }

            if (next == NULL || back == NULL) {
                std::cout << "NULL\n";
                continue;
            }

            GrasperLine g1(current, *next, 1);
            GrasperLine g2(*back, current, 1);
            Eigen::Vector2f normal = 0.5f * (g1.ortogonal() + g2.ortogonal());
            dir = (current.p - centroid)*(1.0f / (current.p - centroid).norm());
            if (normal.dot(dir) < 0)normal = -normal;
            points[i].normal = normal;
            points[i].curvature = acos(g2.ortogonal().dot(g1.ortogonal()));
        }
    }

    bool Grasper::isValidPlanarConfiguration(std::vector<int>& indices) {
        std::vector<GrasperPoint> points;

        for (int i = 0; i < indices.size(); i++) {
            points.push_back(this->points[indices[i]]);
        }
        return isValidPlanarConfiguration(points);
    }

    bool Grasper::isValidPlanarConfiguration(std::vector<GrasperPoint>& points) {
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

    bool Grasper::isVectorPositiveCombinationOf(Eigen::Vector2f& vector, std::vector<GrasperPoint>& points) {

        for (int i = 0; i < points.size(); i++) {
            if (vector.dot(points[i].normal) > 0)return true;
        }
        return false;
    }



}