#include <iostream>
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

bool is_feasible(const Point3d &v, const double coeff_1, const double coeff_2, double w_left, double w[3]) {
    if (v.x < -EPSILON || v.y < -EPSILON || v.z < -EPSILON) {
        return false;
    } else if (v.x + v.y > coeff_1 + EPSILON) {
        return false;
    } else if (v.z > coeff_2 + EPSILON) {
        return false;
    } else if ((w[0] * v.x) + (w[1] * v.y) + (w[2] * v.z) > w_left + EPSILON) {
        return false;
    }

    return true;
}

/**
 * Solve the LP by iterating throught all the hardcoded vertices of the polytope
 */
std::pair<Point3d, double> solve_lp(const ProblemData& problem_data, const State& current_state, int v_idx, double w_left) {
    int n = problem_data.villages[v_idx].population;
    int wet = current_state.villageStates[v_idx].wet_food_rec;
    int dry = current_state.villageStates[v_idx].dry_food_rec;
    int other = current_state.villageStates[v_idx].other_food_rec;

    const double coeff_1 = (9*n) - wet - dry, coeff_2 = n - other;
    
    std::vector<Point3d> candidates = std::vector<Point3d>();
    double v[3], w[3];
    for (int i = 0; i < 3; ++i) {
        v[i] = problem_data.packages[i].value;
        w[i] = problem_data.packages[i].weight;
    }

    // std::cout << "Problem Description:" << std::endl;
    // std::cout << "Dry food rec: " << dry << std::endl;
    // std::cout << "Wet food rec: " << wet << std::endl;
    // std::cout << "Other rec: " << other << std::endl;

    // for (int i = 0; i < 3; ++i) {
    //     std::cout << i << " " << w[i] << " " << v[i] << std::endl;
    // }

    // 1. Intersection of a1=0, a2=0, a3=0
    candidates.push_back({0, 0, 0});

    // 2. Intersection of a1=0, a2=0, a3=n-other
    if (coeff_2 >= 0) { candidates.push_back({0, 0, coeff_2}); }

    // 3. Intersection of a1=0, a2=0, w_3a_3=w'
    if (std::abs(w[2]) > EPSILON) {
        if (w_left <= w[2] * coeff_2) { candidates.push_back({0, 0, w_left / w[2]}); }
    }
    
    // 4. Intersection of a1=0, a3=0, a2=(9n-wet-dry)
    if (coeff_1 >= 0) { candidates.push_back({0, coeff_1, 0}); }
    
    // 5. Intersection of a1=0, a3=0, w_2a_2=w'
    if (std::abs(w[1]) > EPSILON) {
        if (w_left <= w[1] * coeff_1) { candidates.push_back({0, w_left / w[1], 0}); }
    }

    // 6. Intersection of a2=0, a3=0, a1=(9n-wet-dry)
    if (coeff_1 >= 0) { candidates.push_back({coeff_1, 0, 0}); }
    
    // 7. Intersection of a2=0, a3=0, w_1a_1=w'
    if (std::abs(w[0]) > EPSILON) {
        if (w_left <= w[0] * coeff_1) { candidates.push_back({w_left / w[0], 0, 0}); }
    }

    // 8. Intersection of a1=0, a3=n-other, a2=(9n-wet-dry)
    if (coeff_1 >= 0) { candidates.push_back({0, coeff_1, coeff_2}); }

    // 9. Intersection of a1=0, a2=(9n-wet-dry), w_1a_1+w_2a_2+w_3a_3=w'
    if (std::abs(w[2]) > EPSILON && coeff_1 >= 0) {
        double a3 = (w_left - coeff_1*w[1]) / w[2];
        if (a3 >= 0 && a3 <= coeff_2) { candidates.push_back({0, coeff_1, a3}); }
    }

    // 10. Intersection of a2=0, a3=n-other, a1=(9n-wet-dry)
    if (coeff_1 >= 0 && coeff_2 >= 0) { candidates.push_back({coeff_1, 0, coeff_2}); }

    // 11. Intersection of a2=0, a1=(9n-wet-dry), w_1a_1+w_2a_2+w_3a_3=w'
    if (std::abs(w[2]) > EPSILON && coeff_1 >= 0) {
        double a3 = (w_left - coeff_1*w[0]) / w[2];
        if (a3 >= 0 && a3 <= coeff_2) { candidates.push_back({coeff_1, 0, a3}); }
    }
    
    // 12. Intersection of a1=0, a3=n-other, w_1a_1+w_2a_2+w_3a_3=w'
    if (std::abs(w[1]) > EPSILON && coeff_2 >= 0) {
        double a2 = (w_left - coeff_2*w[2]) / w[1];
        if (a2 >= 0 && a2 <= coeff_1) { candidates.push_back({0, a2, coeff_2}); }
    }
    
    // 13. Intersection of a2=0, a3=n-other, w_1a_1+w_2a_2+w_3a_3=w'
    if (std::abs(w[0]) > EPSILON && coeff_2 >= 0) {
        double a1 = (w_left - coeff_2*w[2]) / w[0];
        if (a1 >= 0 && a1 <= coeff_1) { candidates.push_back({a1, 0, coeff_2}); }
    }

    // 14. Intersection of a3=0, a1+a2=(9n-wet-dry), w_1a_1+w_2a_2+w_3a_3=w'
    if (std::abs(w[1] - w[0]) > EPSILON && coeff_1 >= 0) {
        double a2 = (w_left - coeff_1*w[0]) / (w[1] - w[0]);
        double a1 = coeff_1 - a2;
        if (a1 >= 0 && a2 >= 0) { candidates.push_back({a1, a2, 0}); }
    }

    // 15. Intersection of a1+a2=(9n-wet-dry), a3=n-other, w_1a_1+w_2a_2+w_3a_3=w'
    if (std::abs(w[1] - w[0]) > EPSILON && coeff_1 >= 0 && coeff_2 >= 0) {
        double a2 = (w_left - coeff_2*w[2]) - (coeff_1*w[0]) / (w[1] - w[0]);
        double a1 = coeff_1 - a2;
        if (a1 >= 0 && a2 >= 0) { candidates.push_back({a1, a2, coeff_2}); }
    }

    Point3d best_solution = {0.0, 0.0, 0.0};
    bool found_feasible = false;
    double best_objective = 0.0;

    for (const Point3d &candidate : candidates) {
        if (is_feasible(candidate, coeff_1, coeff_2, w_left, w)) {
            found_feasible = true;
            double current_z = (v[0] * candidate.x) + (v[1] * candidate.y) + (v[2] * candidate.z);
            if (current_z > best_objective) {
                best_objective = current_z;
                best_solution = {candidate.x, candidate.y, candidate.z};
            }
        }
    }

    // std::cout << "Candidates" << std::endl;
    // std::cout << "==========" << std::endl;

    // for (auto &c: candidates) {
    //     std::cout << "\t" << c.x << " " << c.y << " " << c.z << std::endl;
    // }

    // std::cout << "Found feasible: " << found_feasible << std::endl;
    // if (found_feasible) std::cout << best_solution.x << " " << best_solution.y << " " << best_solution.z << std::endl;

    return std::pair<Point3d, double>(best_solution, best_objective);
}