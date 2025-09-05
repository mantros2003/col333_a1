#ifndef HELPER_H
#define HELPER_H

#include <vector>
#include <queue>
#include <set>
#include "structures.h"
#include "lp_solver.h"


// IDEA: Don't precompute the supplies being taken on a trip
// Do we even need *_left variables
/** Stores the state of a single helicopter */


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


inline double calculate_total_heli_distance(const HelicopterPlan heli_state, const ProblemData &problem);
double value(State &s, const ProblemData &problem);
std::vector<std::vector<int>> preprocess(const ProblemData &problem);

State expand(State &s, const ProblemData &problem);
std::set<State> expand_single_heli(const State &curr_state, const ProblemData &problem);
std::vector<State> expand_single_heli_stochastic(const State &curr_state, const ProblemData &problem, double (*g)(State), double (*h)(State), int num_samples);
std::vector<State> expand(const State &curr_state, const ProblemData &problem, double (*g)(State), double (*h)(State));

/**
 * Calculates the total distance traveled by a single helicopter in all its trips
 * 
 * Each trip starts at the home city, visits some villages, and may return to the home city
 * @param heli_state The state of a single helicopter.
 * @return The total distance traveled in kilometers.
 */
inline double calculate_total_heli_distance(const HelicopterPlan heli_state, const ProblemData &problem) {
    double total_distance = 0.0;

    Point home_city = problem.cities[problem.helicopters[heli_state.helicopter_id-1].home_city_id-1];
    Point curr_pos = home_city;
    // Iterate through each trip undertaken by the helicopter
    for (const Trip &trip : heli_state.trips) {
        // Iterate through all the drops of the helicopter
        for (const Drop &drop: trip.drops) {
            total_distance += distance(curr_pos, problem.villages[drop.village_id-1].coords);
            curr_pos = problem.villages[drop.village_id-1].coords;
        }
        total_distance += distance(curr_pos, home_city);
    }

    return total_distance;
}

#endif // HELPER_H