#ifndef EXPAND_H
#define EXPAND_H

#include <vector>
#include <queue>
#include <set>
#include <random>
#include "structures.h"
#include "lp_solver.h"


// IDEA: Don't precompute the supplies being taken on a trip
// Do we even need *_left variables
/** Stores the state of a single helicopter */


inline double calculate_total_heli_distance(const HelicopterPlan heli_state, const ProblemData &problem);
double value(State &s, const ProblemData &problem);
std::vector<std::vector<int>> preprocess(const ProblemData &problem);

std::set<State> expand_single_heli(const State &curr_state, const ProblemData &problem);
std::set<State> expand_single_heli_stochastic(const State &curr_state, const ProblemData &problem, int num_samples, std::mt19937&);
std::set<State> expand(const State&, const ProblemData&, int, std::mt19937&);

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

#endif // EXPAND_H