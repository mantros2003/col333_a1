#include <vector>
#include "lp_solver.h"

/** Compute the determinent of a 3*3 matrix */
double det3(double m[3][3]) {
    return m[0][0]*(m[1][1]*m[2][2] - m[1][2]*m[2][1])
         - m[0][1]*(m[1][0]*m[2][2] - m[1][2]*m[2][0])
         + m[0][2]*(m[1][0]*m[2][1] - m[1][1]*m[2][0]);
}

/** Compute the intersection of 3 3d planes */
Point3d intersection_pt(Plane3d p1, Plane3d p2, Plane3d p3) {
    double A[3][3] = {
        {p1.coeff[0], p1.coeff[1], p1.coeff[2]},
        {p2.coeff[0], p2.coeff[1], p2.coeff[2]},
        {p3.coeff[0], p3.coeff[1], p3.coeff[2]}
    };
    double d[3] = {p1.coeff[3], p2.coeff[3], p3.coeff[3]};

    double detA = det3(A);
    if (fabs(detA) < 1e-9) {
        throw std::runtime_error("Planes do not intersect at a single point");
    }

    double Ax[3][3] = {
        {d[0], A[0][1], A[0][2]},
        {d[1], A[1][1], A[1][2]},
        {d[2], A[2][1], A[2][2]}
    };
    double Ay[3][3] = {
        {A[0][0], d[0], A[0][2]},
        {A[1][0], d[1], A[1][2]},
        {A[2][0], d[2], A[2][2]}
    };
    double Az[3][3] = {
        {A[0][0], A[0][1], d[0]},
        {A[1][0], A[1][1], d[1]},
        {A[2][0], A[2][1], d[2]}
    };

    Point3d p;
    p.x = det3(Ax) / detA;
    p.y = det3(Ay) / detA;
    p.z = det3(Az) / detA;
    return p;
}

bool is_feasible(const Point3d &v, double n, double w_left, double w[3]) {
    if (v.x < -EPSILON || v.y < -EPSILON || v.z < -EPSILON) {
        return false;
    } else if (v.x + v.y > 9 * n + EPSILON) {
        return false;
    } else if (v.z > n + EPSILON) {
        return false;
    } else if (w[0] * v.x + w[1] * v.y + w[2] * v.z > w_left + EPSILON) {
        return false;
    }

    return true;
}

/**
 * Solve the LP by iterating throught all the hardcoded vertices of the polytope
 */
std::pair<Point3d, double> solve_lp(double v[3], double w[3], int n, double w_left) {
    std::vector<Point3d> candidates;

    // 1. Intersection of a1=0, a2=0, a3=0
    candidates.push_back({0, 0, 0});

    // 2. Intersection of a1=0, a2=0, a3=n
    candidates.push_back({0, 0, n * 1.0});

    // 3. Intersection of a1=0, a2=0, w_3a_3=w'
    if (std::abs(w[2]) > EPSILON) {
        candidates.push_back({0, 0, w_left / w[2]});
    }
    
    // 4. Intersection of a1=0, a3=0, a2=9n
    candidates.push_back({0, 9.0*n, 0});
    
    // 5. Intersection of a1=0, a3=0, w_2a_2=w'
    if (std::abs(w[1]) > EPSILON) {
        candidates.push_back({0, w_left / w[1], 0});
    }

    // 6. Intersection of a2=0, a3=0, a1=9n
    candidates.push_back({9.0*n, 0, 0});
    
    // 7. Intersection of a2=0, a3=0, w_1a_1=w'
    if (std::abs(w[0]) > EPSILON) {
        candidates.push_back({w_left / w[0], 0, 0});
    }

    // 8. Intersection of a1=0, a3=n, a2=9n
    candidates.push_back({0, 9.0*n, 1.0*n});

    // 9. Intersection of a1=0, a2=9n, w_1a_1+w_2a_2+w_3a_3=w'
    if (std::abs(w[2]) > EPSILON) {
        double a3 = (w_left - 9*n*w[1]) / w[2];
        candidates.push_back({0, 9.0*n, a3});
    }

    // 10. Intersection of a2=0, a3=n, a1=9n
    candidates.push_back({9.0*n, 0, 1.0*n});

    // 11. Intersection of a2=0, a1=9n, w_1a_1+w_2a_2+w_3a_3=w'
    if (std::abs(w[2]) > EPSILON) {
        double a3 = (w_left - 9*n*w[0]) / w[2];
        candidates.push_back({9.0*n, 0, a3});
    }
    
    // 12. Intersection of a1=0, a3=n, w_1a_1+w_2a_2+w_3a_3=w'
    if (std::abs(w[1]) > EPSILON) {
        double a2 = (w_left - n*w[2]) / w[1];
        candidates.push_back({0, a2, 1.0*n});
    }
    
    // 13. Intersection of a2=0, a3=n, w_1a_1+w_2a_2+w_3a_3=w'
    if (std::abs(w[0]) > EPSILON) {
        double a1 = (w_left - n*w[2]) / w[0];
        candidates.push_back({a1, 0, 1.0*n});
    }

    // 14. Intersection of a3=0, a1+a2=9n, w_1a_1+w_2a_2+w_3a_3=w'
    if (std::abs(w[1] - w[0]) > EPSILON) {
        double a2 = (w_left - 9*n*w[0]) / (w[1] - w[0]);
        double a1 = 9*n - a2;
        candidates.push_back({a1, a2, 0});
    }

    // 15. Intersection of a1+a2=9n, a3=n, w_1a_1+w_2a_2+w_3a_3=w'
    if (std::abs(w[1] - w[0]) > EPSILON) {
        double a2 = (w_left - n*w[2] - 9*n*w[0]) / (w[1] - w[0]);
        double a1 = 9*n - a2;
        candidates.push_back({a1, a2, 1.0*n});
    }

    Point3d best_solution;
    bool found_feasible = false;
    double best_objective = -1.0;

    for (const Point3d &candidate : candidates) {
        if (is_feasible(candidate, n, w_left, w)) {
            found_feasible = true;
            double current_z = v[0] * candidate.x + v[1] * candidate.y + v[2] * candidate.z;
            if (current_z > best_objective) {
                best_objective = current_z;
                best_solution = {candidate.x, candidate.y, candidate.z};
            }
        }
    }

    return std::pair<Point3d, double>(best_solution, best_objective);
}