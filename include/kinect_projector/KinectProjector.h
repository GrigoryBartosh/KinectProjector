#pragma once

#include <octomap/octomap.h>
#include <octomap/AbstractOcTree.h>
#include <octomap/math/Pose6D.h>
#include <cassert>
#include <algorithm>
#include <cmath>
#include "kinect_projector/KinectFrame.h"

class KinectProjector
{
private:
    static const double HEIGHT_ANGLE;
    static double PIX_TO_REAL;
    static size_t WIDTH;
    static size_t HEIGHT;
    static const double CUBE_FRAGMENTATION;
    static const double POINT_K;

    struct Point
    {
    private:
        int _x, _y;
    public:
        Point() = default;
        int& x() { return _x; }
        int& y() { return _y; }
    };

    static void init(size_t width, size_t height)
    {
        WIDTH = width;
        HEIGHT = height;
        assert(WIDTH > 0 && HEIGHT * 4 == WIDTH * 3 && "Invalid matrix size");

        PIX_TO_REAL = tan(HEIGHT_ANGLE / 2.0) * 2.0 / HEIGHT;
    }

    static octomap::point3d transform_point(const octomath::Pose6D &T, octomap::point3d v3)
    {
        v3 = v3 - T.trans();
        v3 = T.rot().rotate(v3);
        return v3;
    }

    static Point real_to_pix(octomap::point3d v3)
    {
        Point p;
        p.x() = v3.x() / v3.z() / PIX_TO_REAL + WIDTH  / 2;
        p.y() = v3.y() / v3.z() / PIX_TO_REAL + HEIGHT / 2;
        return p;
    }

    static void split_cube(octomap::point3d cube, double len, const octomath::Pose6D &Transform, KinectFrame &projection)
    {
        double board = len / 2.0;
        double step  = board / (int)(board / CUBE_FRAGMENTATION);

        for (int shift = 0; shift < 3; shift++)
        for (int a_ = -1; a_ <= 1; a_ += 2)
        for (double b = -board; b < board; b += step)
        for (double c = -board; c < board; c += step)
        {
            double a = a_ * board;
            double dx = a;
            double dy = b;
            double dz = c;
            for (int i = 0; i < shift; i++)
            {
                std::swap(dx, dy);
                std::swap(dy, dz);
            }

            octomap::point3d v3(cube.x() + dx, cube.y() + dy, cube.z() + dz);
            v3 = transform_point(Transform, v3);
            Point v2 = real_to_pix(v3);
            projection.update(v2.x(), v2.y(), v3.z() * POINT_K);
        }
    }

    static void process_cubes(const octomap::OcTree &tree, const octomath::Pose6D &Transform, KinectFrame &projection)
    {
        for (octomap::OcTree::leaf_iterator it = tree.begin_leafs(); it != tree.end_leafs(); it++)
            split_cube(it.getCoordinate(), it.getSize(), Transform, projection);
    }

public:
    KinectProjector() = delete;

    static void project(const octomap::OcTree &tree, const octomath::Pose6D &Transform, KinectFrame &projection)
    {
        init(projection.width(), projection.height());
        process_cubes(tree, Transform, projection);
    }
};

const double KinectProjector::HEIGHT_ANGLE = 2.0 * M_PI / 9.0; // 40 degrees
double KinectProjector::PIX_TO_REAL = 0;
size_t KinectProjector::WIDTH = 0;
size_t KinectProjector::HEIGHT = 0;
const double KinectProjector::CUBE_FRAGMENTATION = 0.005;
const double KinectProjector::POINT_K = 100.0;