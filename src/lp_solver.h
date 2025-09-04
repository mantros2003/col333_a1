#ifndef LP_SOLVER_H
#define LP_SOVER_H

#include <utility>

#include "structures.h"

const double EPSILON = 1e-9;

struct Point3d {
    double x, y, z;
};

struct Plane3d {
    double coeff[4];
};

// 0 is for >=, 1 is for <=
struct Constraint3d {
    Plane3d c_plane;
    bool kind;
};

double det3(double m[3][3]);
Point3d intersection_pt(Plane3d p1, Plane3d p2, Plane3d p3);
bool is_feasible(const Point3d &v, double n, double w_left, double w[3]);
std::pair<Point3d, double> solve_lp(double v[3], double w[3], int n, double w_left);

#endif // LP_SOLVER_H