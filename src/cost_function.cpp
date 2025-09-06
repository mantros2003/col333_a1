#include "structures.h"
#include "lp_solver.h"
#include "vector"

#include <cmath>
#include <limits>
#include <utility>
#include <algorithm>
#include <stack>
#include <iostream>
#include <chrono>
// #include <cmath>

using namespace std;
// You can add any helper functions or classes you need here.

/**
 * @brief The main function to implement your search/optimization algorithm.
 * * This is a placeholder implementation. It creates a simple, likely invalid,
 * plan to demonstrate how to build the Solution object. 
 * * TODO: REPLACE THIS ENTIRE FUNCTION WITH YOUR ALGORITHM.
 */

// double dis(Point a, Point b){
//     return sqrt(pow((a.x-b.x),2) + pow((a.y-b.y),2));
// }

// Project point p onto unit direction (cx, sy) and return scalar coordinate.
double proj(const Point& p, double cx, double sy) {
    return p.x * cx + p.y * sy;
}

// Nearest distance from anchor index 'a' into the unvisited set U.
double nearest_into_set(int a, const vector<int>& v_idx, const ProblemData& P) {
    if (v_idx.empty()) return 0.0;
    double best = numeric_limits<double>::infinity();
    for (int u : v_idx) {
        double d = distance(P.villages[a].coords, P.villages[u].coords);
        if (d < best) best = d;
    }
    return best;
}

// 2 * projection span over a small set of directions, including the start s.
// Using four directions: 0°, 45°, 90°, 135°.
double closed_tour_span2(const vector<int>& v_idx, int s, const ProblemData& P) {
    if (v_idx.empty()) return 0.0;

    // cos/sin pairs for 0°, 45°, 90°, 135°
    constexpr double SQRT1_2 = 0.70710678118654752440; // 1/sqrt(2)
    const double C[4] = { 1.0,  SQRT1_2, 0.0, -SQRT1_2 };
    const double S[4] = { 0.0,  SQRT1_2, 1.0,  SQRT1_2 };

    double best_span2 = 0.0;

    for (int k = 0; k < 4; ++k) {
        double mn =  numeric_limits<double>::infinity();
        double mx = -numeric_limits<double>::infinity();
        // include all unvisited plus the start to enforce closed-tour variation
        for (int u : v_idx) {
            double p = proj(P.villages[u].coords, C[k], S[k]);
            if (p < mn) mn = p;
            if (p > mx) mx = p;
        }
        {
            double ps = proj(P.villages[s].coords, C[k], S[k]);
            if (ps < mn) mn = ps;
            if (ps > mx) mx = ps;
        }
        double span2 = 2.0 * (mx - mn);   // closed tour needs to traverse the projection twice
        if (span2 > best_span2) best_span2 = span2;
    }

    return best_span2;
}

// Admissible heuristic for a closed tour: start at s, current at c, must visit all in v_index and return to s.
double distance_travelled(int s, const vector<int>& v_idx, const ProblemData& P) {
    if (v_idx.empty()) {
        // Only need to return home.
        return 0.0;
    }

    // 1) Lower bound from closed-tour projection spans (rotation-aware, memory-lean)
    double span2 = closed_tour_span2(v_idx, s, P);

    // 2) Must at least enter U from current, and
    // 3) must also reconnect to start from somewhere in U.
    // double enter  = nearest_into_set(c, v_idx, P);
    double anchor = nearest_into_set(s, v_idx, P);

    // Take the maximum of necessary independent lower bounds to stay admissible.
    return max(span2,anchor);
}


// Prim’s MST (O(n^2)) rooted at s. Returns {parent, total_weight}.
// parent[s] == -1. Assumes complete Euclidean graph on P.
inline std::pair<std::vector<int>, double>
prim_mst(const std::vector<Point>& P, int s) {
    const int n = (int)P.size();
    const double INF = std::numeric_limits<double>::infinity();
    std::vector<double> key(n, INF);
    std::vector<int> parent(n, -1);
    std::vector<char> in(n, 0);

    key[s] = 0;
    double total = 0;

    for (int it = 0; it < n; ++it) {
        int u = -1; double best = INF;
        for (int i = 0; i < n; ++i)
            if (!in[i] && key[i] < best) { best = key[i]; u = i; }
        if (u == -1) break; // should not happen for complete graphs

        in[u] = 1;
        if (parent[u] != -1) total += distance(P[u], P[parent[u]]);

        for (int v = 0; v < n; ++v) if (!in[v]) {
            double w = distance(P[u], P[v]);
            if (w < key[v]) { key[v] = w; parent[v] = u; }
        }
    }
    return {parent, total};
}

// Build MST adjacency from parent array (undirected), with Euclidean weights.
inline std::vector<std::vector<std::pair<int,double>>>
build_mst_graph(const std::vector<Point>& P, const std::vector<int>& parent) {
    const int n = (int)P.size();
    std::vector<std::vector<std::pair<int,double>>> g(n);
    for (int v = 0; v < n; ++v) if (parent[v] != -1) {
        int u = parent[v];
        double w = distance(P[u], P[v]);
        g[u].push_back({v, w});
        g[v].push_back({u, w});
    }
    return g;
}

// Tree distances from root s along the MST (sum of edge weights on unique paths).
inline std::vector<double>
mst_tree_distances_from_root(const vector<vector<pair<int,double>>>& g, int s) {
    const int n = (int)g.size();
    std::vector<double> dist(n, -1);
    std::stack<int> st;
    st.push(s);
    dist[s] = 0;
    while (!st.empty()) {
        int u = st.top(); st.pop();
        for (pair<int,double> i : g[u]) if (dist[i.first] < 0) {
            int v = i.first;
            double w = i.second;
            dist[v] = dist[u] + w;
            st.push(v);
        }
    }
    return dist;
}

// Degree-based lower bound: 0.5 * sum over i of the two smallest incident distances from i.
// Safe and usually stronger than plain MST weight. O(n^2).
inline double
degree_lower_bound(const std::vector<Point>& P) {
    const int n = (int)P.size();
    if (n <= 2) { // trivial cases
        return (n <= 1) ? 0.0 : distance(P[0], P[1]) * 1.0;
    }
    const double INF = std::numeric_limits<double>::infinity();
    double sum_two_smallest = 0.0;
    for (int i = 0; i < n; ++i) {
        double a = INF, b = INF;
        for (int j = 0; j < n; ++j) if (j != i) {
            double d = distance(P[i], P[j]);
            if (d < a) { b = a; a = d; }
            else if (d < b) { b = d; }
        }
        sum_two_smallest += (a + b);
    }
    return 0.5 * sum_two_smallest;
}

struct Bounds {
    double lower;       // lower bound on minimal distance
    double upper;       // upper bound on minimal distance
    double mst_weight;  // MST weight used
    int    ub_endpoint; // index u that achieves the upper bound
};

// Compute tight, fast bounds on the minimal tour length where you start at s,
// visit all points at least once, end anywhere, then add straight-line return to s.
// Returns lower/upper bounds and the MST weight. O(n^2).
inline Bounds
tsp_min_distance_bounds(const ProblemData& problem,const vector<int>& v_idx, int s) {
    vector<Point> P;
    for (int v = 0; v < v_idx.size(); v++){
        P.push_back(problem.villages[v].coords);
    }
    const int n = (int)P.size();
    if (n == 0 || n == 1) {
        return {0.0, 0.0, 0.0, (n ? s : -1)};
    }
    // 1) MST
    pair<vector<int>, double> prim = prim_mst(P, s);
    vector<int> parent = prim.first;
    double W_mst = prim.second;
    auto g = build_mst_graph(P, parent);

    // 2) Tree distances from s
    auto dist_tree = mst_tree_distances_from_root(g, s);

    // 3) Lower bound
    double LB_degree = degree_lower_bound(P);
    double LB = std::max(W_mst, LB_degree);

    // 4) Improved upper bound: walk MST (DFS) ending at u, then straight-line hop u->s
    // Cost(candidate u): 2*W_mst - dist_tree[u] + euclid(P[u], P[s])
    double UB = std::numeric_limits<double>::infinity();
    int best_u = s;
    for (int u = 0; u < n; ++u) {
        if (dist_tree[u] < 0) continue; // disconnected should not occur
        double cand = 2.0 * W_mst - dist_tree[u] + distance(P[u], P[s]);
        if (cand < UB) { UB = cand; best_u = u; }
    }

    return {LB, UB, W_mst, best_u};
}


double g(int village_index, int city_index, int helicopter_index, const ProblemData& problem_data,const State& current_state){
    // caluclate the cost from the start to this village.
    int fixed_cost = problem_data.helicopters[helicopter_index].fixed_cost;
    int alpha = problem_data.helicopters[helicopter_index].alpha;
    double distance_travelled = distance(problem_data.villages[village_index].coords, problem_data.cities[city_index]);
    int v_population = problem_data.villages[village_index].population;

    double weight_cap = problem_data.helicopters[helicopter_index].weight_capacity;

    double fuel_cost = 2*fixed_cost + 2*alpha*distance_travelled;
    double value_cost = solve_lp(problem_data, current_state,v_population, weight_cap).second;
    if (value_cost-2*fuel_cost<=0) {return INT_MIN*1.0;}
    double prev_state_cost = current_state.g_cost;
    return (prev_state_cost + value_cost - fuel_cost);
}

double h(int helicopter_index, int curr_village_idx, const ProblemData& problem_data, const State& current_state){
    vector<int> v_index; //index of all the villages that require help
    
    vector<V_state> village_states = current_state.villageStates;
    // vector<Village> villages = problem_data.villages;
    
    for (int i = 0; i < village_states.size(); i++){
        if (village_states[i].help_needed){v_index.push_back(i);}
    }
    
    double wet_count = 0.0, other_count = 0.0;
    for (int i = 0; i < v_index.size(); i++){
        wet_count += (village_states[i].dry_food_rec + village_states[i].wet_food_rec);
        other_count += village_states[i].other_food_rec;
    }
    double distance = distance_travelled(curr_village_idx, v_index, problem_data);
    // double distance = tsp_min_distance_bounds(problem_data, v_index, curr_village_idx).lower;
    double fixed_cost, alpha;
    fixed_cost = problem_data.helicopters[helicopter_index].fixed_cost;
    alpha = problem_data.helicopters[helicopter_index].alpha;
    double fuel_cost = fixed_cost + alpha * distance;
    double gross_value = problem_data.packages[1].value * wet_count + problem_data.packages[2].value * other_count;
    double h_cost = gross_value - fuel_cost;

    return h_cost;
}



