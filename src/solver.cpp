#include "solver.h"
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

double dis(Point a, Point b){
    return sqrt(pow((a.x-b.x),2) + pow((a.y-b.y),2));
}

// Project point p onto unit direction (cx, sy) and return scalar coordinate.
double proj(const Point& p, double cx, double sy) {
    return p.x * cx + p.y * sy;
}

// Nearest distance from anchor index 'a' into the unvisited set U.
double nearest_into_set(int a, const vector<int>& v_idx, const ProblemData& P) {
    if (v_idx.empty()) return 0.0;
    double best = numeric_limits<double>::infinity();
    for (int u : v_idx) {
        double d = dis(P.villages[a].coords, P.villages[u].coords);
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

double g(int village_index, int city_index, int helicopter_index, const ProblemData& problem_data, State& current_state){
    // caluclate the cost from the start to this village.
    int fixed_cost = problem_data.helicopters[helicopter_index].fixed_cost;
    int alpha = problem_data.helicopters[helicopter_index].alpha;
    double distance_travelled = dis(problem_data.villages[village_index].coords, problem_data.cities[city_index]);
    int v_population = problem_data.villages[village_index].population;
    double dry_weight, dry_value; int dry_count;
    double wet_weight, wet_value; int wet_count;
    double other_weight, other_value;
    double weight_cap = problem_data.helicopters[helicopter_index].weight_capacity;
    dry_weight = problem_data.packages[0].weight;
    dry_value = problem_data.packages[0].value;
    wet_weight = problem_data.packages[1].weight;
    wet_value = problem_data.packages[1].value;
    other_weight = problem_data.packages[2].weight;
    other_value = problem_data.packages[2].value;
    wet_count = (weight_cap-v_population*(other_weight+9*dry_weight))/(wet_weight-dry_weight);
    dry_count = 9*v_population-wet_count;

    double fuel_cost = 2*fixed_cost + 2*alpha*distance_travelled;
    double value_cost = wet_value*wet_count + dry_value*dry_count + other_value*v_population;
    double prev_state_cost = current_state.g_cost;
    return prev_state_cost + value_cost - fuel_cost;
}

double h(int helicopter_index, int curr_village_idx, const ProblemData& problem_data, State& current_state){
    vector<int> v_index; //index of all the villages that require help
    
    vector<V_state> village_states = current_state.v_states;
    // vector<Village> villages = problem_data.villages;
    
    for (int i = 0; i < village_states.size(); i++){
        if (village_states[i].help_needed){v_index.push_back(i);}
    }
    
    double wet_count, other_count;
    for (int i = 0; i < v_index.size(); i++){
        wet_count += 9*village_states[i].population - village_states[i].dry_food_count - village_states[i].wet_food_count;
        other_count += village_states[i].population - village_states[i].other_supplies_count;
    }
    double distance = distance_travelled(curr_village_idx, v_index, problem_data);
    double fixed_cost, alpha;
    fixed_cost = problem_data.helicopters[helicopter_index].fixed_cost;
    alpha = problem_data.helicopters[helicopter_index].alpha;
    double fuel_cost = fixed_cost + alpha * distance;
    double gross_value = problem_data.packages[1].value * wet_count + problem_data.packages[2].value * other_count;
    double h_cost = gross_value - fuel_cost;

    return h_cost;
}



Solution solve(const ProblemData& problem) {
    cout << "Starting solver..." << endl;

    Solution solution;

    // --- START OF PLACEHOLDER LOGIC ---
    // This is a naive example: send each helicopter on one trip to the first village.
    // This will definitely violate constraints but shows the structure.
    
    for (const auto& helicopter : problem.helicopters) {
        HelicopterPlan plan;
        plan.helicopter_id = helicopter.id;

        if (!problem.villages.empty()) {
            Trip trip;
            // Pickup 1 of each package type
            trip.dry_food_pickup = 1;
            trip.perishable_food_pickup = 1;
            trip.other_supplies_pickup = 1;

            // Drop them at the first village
            Drop drop;
            drop.village_id = problem.villages[0].id;
            drop.dry_food = 1;
            drop.perishable_food = 1;
            drop.other_supplies = 1;

            trip.drops.push_back(drop);
            plan.trips.push_back(trip);
        }
        solution.push_back(plan);
    }
    
    // --- END OF PLACEHOLDER LOGIC ---

    cout << "Solver finished." << endl;
    return solution;
}