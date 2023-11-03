#ifndef GRAHAMSCANCONVEXHULL_H
#define GRAHAMSCANCONVEXHULL_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <iostream>
#include <stack>
#include <stdlib.h>

using namespace std;
using namespace Eigen;

class GrahamScanConvexHull
{
    public:

        GrahamScanConvexHull();

        //TODO: add set of points on which compute the convex hull
        void add_set_of_points(vector<Vector3d>& points_set);

        //TODO: get convex hull
        void convex_hull(vector<Vector3d>& points_set);

    private:

        vector<Vector3d> _points_set;

        Vector3d _pivot;

        Vector3d next_to_top(stack<Vector3d>& stack);

        void swap(Vector3d& p1, Vector3d& p2);

        /**
         * @brief Compute the square distance between two points
         *
         * dist = √((x1 - x2)² + (y1 - y2)² + (z1 - z2)²)
         *
         * @param p1 point 1
         * @param p2 point 2
         * @return the distance without the square, because it is used for comparison purpose.
         */
        double square_distance(Vector3d& p1, Vector3d& p2);

        double orientation(Vector3d pivot, Vector3d p1, Vector3d p2);

        bool compare(Vector3d p1, Vector3d p2);


};

#endif // GRAHAMSCANCONVEXHULL_H
