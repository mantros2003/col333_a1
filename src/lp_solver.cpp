#include <vector>

double solve_lp(std::vector<double> v, std::vector<double> w, int n, double w_left) {
    double a3_max = std::min((double) n, w_left / (w[2] + 1e-9));

    double r = w_left - (w[2] * a3_max);
}