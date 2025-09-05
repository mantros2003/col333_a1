#ifndef HELPER_H
#define HELPER_H

#include <vector>
#include <queue>
#include <set>
#include "structures.h"
#include "lp_solver.h"

// Remove this Trip Struct, use the given struct
// struct Trip {
//     vector<int> villages;
//     bool is_complete;
//     double distance_covered;
//     int supplies_distributed[3];
// };

// IDEA: Don't precompute the supplies being taken on a trip
// Do we even need *_left variables
/** Stores the state of a single helicopter */

// Remove this helicopter state and use the provided Heliopter state
// struct HeliState {
//     Helicopter* heli;
//     int dry_food_left;
//     int wet_food_left;
//     int other_food_left;
//     // Vector of vector of ints
//     // For each trip, a different vector is created
//     // Each vector has the village id in order of visits
//     vector<Trip> trips;
// };

/** Stores the state of a single village */
struct V_state {
    // Village vill;
    bool help_needed;
    int dry_food_rec;
    int wet_food_rec;
    int other_food_rec;
};

/** Combines helicopter and village states */
struct State {
    double g_cost, h_cost;
    vector<HelicopterPlan> heliStates;
    vector<V_state> villageStates;

    bool operator>(const State& other) const {
        return (g_cost + h_cost) > (other.g_cost + other.h_cost);
    }
};

//  Remove this Helicopter State
// struct H_state {
//     int home_city_count;
//     int dry_food_count;
//     int wet_food_count;
//     int other_supplies_count;
//     double weight_capacity;
//     double distance_capacity;
//     double distance_max_capacity;
// };

// struct V_state {    
//     bool help_needed;
//     int dry_food_count;
//     int wet_food_count;
//     int other_supplies_count;
//     // int population;
// };

// struct State {
//     double g_cost, h_cost;
//     vector<H_state> h_states;
//     vector<V_state> v_states;
// };


inline double calculate_total_heli_distance(const HeliState& heli_state, const ProblemData &problem);
double value(State &s, const ProblemData &problem);
bool is_village_satisfied(const State &s, int v);
std::vector<std::vector<int>> preprocess(const ProblemData &problem);

State expand(State &s, const ProblemData &problem);
std::vector<Node> expand_single_heli(const Node &parent_node, const ProblemData &problem, double (*g)(State), double (*h)(State));
std::vector<Node> expand(const Node &parent_node, const ProblemData &problem, double (*g)(State), double (*h)(State));

inline double calculate_total_heli_distance(const HeliState& heli_state, const ProblemData &problem) {
    double total_distance = 0.0;
    for (const Trip &trip : heli_state.trips) {
        total_distance += trip.distance_covered;
    }
    return total_distance;
}

#endif // HELPER_H