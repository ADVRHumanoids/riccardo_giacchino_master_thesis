#include "grahamscanconvexhull.h"

using namespace std;
using namespace Eigen;


GrahamScanConvexHull::GrahamScanConvexHull()
{
    _pivot = Vector3d::Zero();
    _points_set = vector<Vector3d>(1, Vector3d::Zero());
}

void GrahamScanConvexHull::add_set_of_points(vector<Vector3d>& points_set)
{
    // Check for more than three elements
    if (points_set.size() < 3){
        string exception = string("convex:hull(): Trying to compute the convex hull with") + to_string(points_set.size()) +
                           string("which is less than 3\n");

        throw std::invalid_argument(exception);

    } else
        _points_set = points_set;

}

Vector3d GrahamScanConvexHull::next_to_top(stack<Vector3d>& stack)
{

    Vector3d top_element = stack.top(); // Take the top element of the stack
    stack.pop();    // pop the stack to expose the next element
    Vector3d second_top_element = stack.top(); // save actual top element after the pop
    stack.push(top_element); // bush bask the original top element to restore the original state of the stack
    return second_top_element;

}

void GrahamScanConvexHull::swap(Vector3d& p1, Vector3d& p2)
{
    Vector3d temp = p1;
    p1 = p2;
    p2 = temp;
}

double GrahamScanConvexHull::square_distance (Vector3d& p1, Vector3d& p2)
{

    double distance = std::pow(p1.x() - p2.x(), 2) +
                      std::pow(p1.y() - p2.y(), 2) +
                      std::pow(p1.z() - p2.z(), 2);
    return distance;

}

double GrahamScanConvexHull::orientation(Vector3d pivot, Vector3d p1, Vector3d p2)
{
    double val = (p1.y() - pivot.y()) * (p2.x() - p1.x()) -
                 (p1.x() - pivot.x()) * (p2.y() - p1.y());

    return val;

}

bool GrahamScanConvexHull::compare(Vector3d p1, Vector3d p2)
{

    double o = orientation(_pivot, p1, p2);

    if (o == 0)
        // If points have the same polar angle, choose the one closer to pivot
        return square_distance(p1, _pivot) <= square_distance(p2, _pivot);

    return o > 0;

}

bool compare_lowest(Vector3d& p1, Vector3d& p2)
{
    return (abs(p1.y()) < abs(p2.y()) ||
            (abs(p1.y()) == abs(p2.y()) && abs(p1.x()) < abs(p2.x())));
}


// Prints convex hull of a set of n points.
void GrahamScanConvexHull::convex_hull(vector<Vector3d>& points_set)
{
    // Find the bottommost point
    _pivot = *min_element(_points_set.begin(), _points_set.end(), compare_lowest);

    // Swap the first element and the pivot point in the vector
    swap(_points_set[0], _pivot);

    sort(_points_set.begin(), _points_set.end(), [this](const Vector3d& a, const Vector3d& b) {
            return compare(a, b);
            });

    int m = 1; // Initialize size of modified array

    for (int i=1; i<_points_set.size(); i++)
    {
        // Keep removing i while angle of i and i+1 is same
        // with respect to p0
        while (i < _points_set.size()-1 && orientation(_pivot, _points_set[i],
                                        _points_set[i+1]) == 0)
            i++;
        _points_set[m] = _points_set[i];
        m++;  // Update size of modified array
    }
    // If modified array of points has less than 3 points,
    // convex hull is not possible
    if (m < 3) return;
    // Create an empty stack and push first three points
    // to it.
    stack<Vector3d> S;
    S.push(_points_set[0]);
    S.push(_points_set[1]);
    S.push(_points_set[2]);


    // Process remaining n-3 points
    for (int i = 3; i < m; i++)
    {
        // Keep removing top while the angle formed by
        // points next-to-top, top, and points[i] makes
        // a non-left turn
        while (orientation(next_to_top(S), S.top(), _points_set[i]) != 2)
            S.pop();

        S.push(_points_set[i]);
    }

    // Convert stack into vector

    while(!S.empty()){
        points_set.push_back(S.top());
        S.pop();
    }

}

