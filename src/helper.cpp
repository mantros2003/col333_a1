#include <iostream>
#include <vector>
#include <queue>

#include "structures.h"

/**
 * Stores the state of a single helicopter
 */
struct HeliState {
    Helicopter heli;
    int dry_food_left;
    int wet_food_left;
    int other_food_left;
    // Vector of vector of ints
    // For each trip, a different vector is created
    // Each vector has the village id in order of visits
    vector<vector<int>> trips;
};

/**
 * Stores the state of a single village
 */
struct VillageState {
    Village vill;
    int dry_food_rec;
    int wet_food_rec;
    int other_food_rec;
};

struct State {
    vector<HeliState> heliStates;
    vector<VillageState> villageStates;
};

// Need to change state
// States need to store order and values of drops to implement capping logic
// Need to cap supplies of each village
double value(State &s, const ProblemData &problem) {
    double value = 0.0;

    // Iterate over all the helicopter's states
    for (auto &heli_state: s.heliStates) {
        // Subtract the total take-off and landing cost (fixed-cost)
        value -= (heli_state.heli.fixed_cost * heli_state.trips.size());

        Point home_city = problem.cities[heli_state.heli.home_city_id-1];
        // Calculate the total distance covered by helicopter over all the trips
        double total_trip_dist = 0.0;
        for (auto &trip: heli_state.trips) {
            for (int i = 0; i < trip.size(); i++) {
                // If first village, then distance from home, else from previous village
                total_trip_dist += distance(problem.villages[i].coords, i ? problem.villages[i-1].coords : home_city);
            }
            // Distance from last village to home city
            total_trip_dist += distance(problem.villages[problem.villages.size()-1].coords, home_city);
        }

        value -= (total_trip_dist * heli_state.heli.alpha);
    }

    double w_d, w_w, w_o;   // Weights
    double v_d, v_w, v_o;   // Values

    w_d = problem.packages[0].weight;
    w_w = problem.packages[1].weight;
    w_o = problem.packages[2].weight;

    v_d = problem.packages[0].value;
    v_d = problem.packages[1].value;
    v_d = problem.packages[2].value;

    // Iteratw over all village states
    for (auto &village_state: s.villageStates) {
        value += (v_d * village_state.dry_food_rec + v_w * village_state.wet_food_rec + v_o * village_state.other_food_rec);
    }

    return value;
}

/**
 * Check if the village's needs are fulfilled
 * @param s The state
 * @param v The village id
 */
bool is_village_satisfied(State &s, int v) {
    VillageState state = s.villageStates[v-1];
    int village_pop = state.vill.population;

    return ((state.dry_food_rec + state.wet_food_rec) >= 9 * village_pop) & (state.other_food_rec >= village_pop)
}

// TODO: Move the function to solver.cpp
State expand(State &s, const ProblemData &problem) {
    int num_helicopters = s.heliStates.size();
    priority_queue<int, vector<int>, greater<int>> min_pq;

    for (int i = 0; i < num_helicopters; ++i) {
        
    }
}

vector<vector<int>> preprocess(const ProblemData &problem) {
    int num_cities = problem.cities.size();
    int num_villages = problem.villages.size();
    int num_helicopters = problem.helicopters.size();

    vector<vector<int>> reachable_by_heli(num_cities, vector<int>());
    for (auto &heli: problem.helicopters) {
        for (int v_idx = 0; v_idx < num_villages; v_idx++) {
            if (distance(problem.cities[heli.home_city_id-1], problem.villages[v_idx].coords) <= heli.distance_capacity) {
                reachable_by_heli[heli.home_city_id-1].push_back(v_idx);
            }
        }
    }

    return reachable_by_heli;
}