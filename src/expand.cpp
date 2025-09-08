#include <iostream>
#include <vector>
#include <queue>
#include <set>
#include <random>
#include <cassert>
#include "cost_function.h"
#include "structures.h"
#include "expand.h"

// Need to change state
// States need to store order and values of drops to implement capping logic
// Need to cap supplies of each village
double value(State &s, const ProblemData &problem) {
    double value = 0.0;

    // Iterate over all the helicopter's states
    for (auto &heli_state: s.heliStates) {
        Helicopter heli = problem.helicopters[heli_state.helicopter_id-1];
        // Subtract the total take-off and landing cost (fixed-cost)
        value -= (heli.fixed_cost * heli_state.trips.size());

        // Calculate the total distance covered by helicopter over all the trips
        double total_trip_dist = calculate_total_heli_distance(s.heliStates[heli_state.helicopter_id-1], problem);

        value -= (total_trip_dist * heli.alpha);
    }

    double v_d, v_w, v_o;   // Values

    v_d = problem.packages[0].value;
    v_w = problem.packages[1].value;
    v_o = problem.packages[2].value;

    // Iteratw over all village states
    for (auto &village_state: s.villageStates) {
        value += (v_d * village_state.dry_food_rec + v_w * village_state.wet_food_rec + v_o * village_state.other_food_rec);
    }

    return value;
}

/**
 * Preprocesses the input and precomputes useful information
 * 
 * Currently computes villages reachable by a helicopter
 * 
 * TODO: Add more things that can be cached and precomputed
 * @param problem Problem data
 * @returns All villages reachable by a helicopter
 */
vector<vector<int>> preprocess(const ProblemData &problem) {
    int num_cities = problem.cities.size();
    int num_villages = problem.villages.size();

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

// Function for single helicopter single village trip expand
std::set<State> expand_single_heli(const State &curr_state, const ProblemData &problem) {
    std::set<State> successors;

    double values[3], weights[3];
    for (int i = 0; i < 3; ++i) {
        values[i] = problem.packages[i].value;
        weights[i] = problem.packages[i].weight;
    }

    // Store the villages that need more supplies
    std::vector<int> villages_left;
    for (size_t v = 0; v < problem.villages.size(); ++v) {
        if (curr_state.villageStates[v].help_needed) { villages_left.push_back(v); }
    }
    // Iterate through each helicopter
    for (size_t i = 0; i < problem.helicopters.size(); ++i) {
        HelicopterPlan heli_state = curr_state.heliStates[i];
        Helicopter heli = problem.helicopters[heli_state.helicopter_id-1];
        Point home_city = problem.cities[heli.home_city_id-1];

        for (int v: villages_left) {
            // Check if heli can travel to the village and come back
            double travel_dist = 2*distance(home_city, problem.villages[v].coords);
            if (travel_dist <= heli.distance_capacity && travel_dist <= curr_state.heliStates[heli.id-1].d_max_left) {
                // // Allocate supplies
                // std::pair<Point3d, double> allocation = solve_lp(problem, curr_state, v, heli.weight_capacity);
                
                // // Duplicate parent state, add a trip and then modify the village's state
                // State child_state = curr_state;
                // child_state.heliStates[heli.id-1].d_max_left -= travel_dist;
                // // Initialize a new trip
                // /** TODO: Check the order of dry, perishable, other*/

                // int dry_rec = child_state.villageStates[v].dry_food_rec;
                // int wet_rec = child_state.villageStates[v].wet_food_rec;
                // int other_rec = child_state.villageStates[v].other_food_rec;
                // if (dry_rec + wet_rec + allocation.first.x + allocation.first.y > 9*problem.villages[v].population){
                //     if (dry_rec + wet_rec + allocation.first.y > 9*problem.villages[v].population){
                //         child_state.villageStates[v].wet_food_rec += 9*problem.villages[v].population-(dry_rec + wet_rec);
                //     }
                //     else {
                //         child_state.villageStates[v].wet_food_rec += allocation.first.y;
                //         child_state.villageStates[v].dry_food_rec += 9*problem.villages[v].population-(dry_rec + wet_rec);
                //     }
                    
                // }
                // else {
                //     child_state.villageStates[v].dry_food_rec += allocation.first.x;
                //     child_state.villageStates[v].wet_food_rec += allocation.first.y;
                // }
                // if (child_state.villageStates[v].other_food_rec > problem.villages[v].population){
                //     child_state.villageStates[v].other_food_rec = problem.villages[v].population;
                // }
                // else {
                //     child_state.villageStates[v].other_food_rec += allocation.first.z;
                // }
                // Trip t;
                // t.dry_food_pickup = allocation.first.x;
                // t.perishable_food_pickup = allocation.first.y;
                // t.other_supplies_pickup = allocation.first.z;
                // Drop d = {v+1, child_state.villageStates[v].dry_food_rec - dry_rec, child_state.villageStates[v].wet_food_rec - wet_rec, child_state.villageStates[v].other_food_rec - other_rec};
                // t.drops = vector<Drop>(1, d);
                // child_state.heliStates[i].trips.push_back(t);
                
                // // Modify the village's state
                // // condition for village's needs
                // if (child_state.villageStates[v].dry_food_rec + child_state.villageStates[v].wet_food_rec > 8.1 * problem.villages[v].population){
                //     child_state.villageStates[v].help_needed = false;
                // }
                // child_state.g_cost = g(v,heli.home_city_id-1,heli_state.helicopter_id-1, problem, curr_state);
                // child_state.h_cost = h(heli_state.helicopter_id-1, v, problem, curr_state);
                // successors.insert(child_state);

                // Duplicate parent state, add a trip and then modify the village's state
                State child_state = curr_state;

                int dry_rec = child_state.villageStates[v].dry_food_rec;
                int wet_rec = child_state.villageStates[v].wet_food_rec;
                int other_rec = child_state.villageStates[v].other_food_rec;

                int meal_req = (9 * problem.villages[v].population) - dry_rec - wet_rec;
                int other_req = problem.villages[v].population - other_rec;

                // Allocate supplies
                std::pair<Point3d, double> allocation = solve_lp(weights, values, meal_req, other_req, heli.weight_capacity);

                // If no allocation, then skip
                if ((allocation.first.x < EPSILON) && (allocation.first.y < EPSILON) && (allocation.first.z < EPSILON)) { continue; }


                // Initialize a new trip
                /** TODO: Check the order of dry, perishable, other*/
                Trip t;
                t.dry_food_pickup = allocation.first.x;
                t.perishable_food_pickup = allocation.first.y;
                t.other_supplies_pickup = allocation.first.z;
                Drop d = {v+1, (int)allocation.first.x, (int)allocation.first.y, (int)allocation.first.z};
                t.drops = vector<Drop>(1, d);
                child_state.heliStates[i].trips.push_back(t);
                child_state.heliStates[i].d_max_left -= travel_dist;

                // Modify the village's state
                child_state.villageStates[v].dry_food_rec += allocation.first.x;
                child_state.villageStates[v].wet_food_rec += allocation.first.y;
                child_state.villageStates[v].other_food_rec += allocation.first.z;

                if (child_state.villageStates[v].dry_food_rec + child_state.villageStates[v].wet_food_rec >= 9 * problem.villages[v].population &&
                    child_state.villageStates[v].other_food_rec >= problem.villages[v].population) {
                    child_state.villageStates[v].help_needed = false;
                }

                child_state.g_cost = g(v, heli.home_city_id-1, heli_state.helicopter_id-1, problem, curr_state, 1.0);
                // child_state.h_cost = h(heli_state.helicopter_id-1, v, problem, curr_state);
                child_state.h_cost = 0;

                successors.insert(child_state);
            }
        }
    }
    return successors;
}

// Same as the above function, just added stochasticity to the resource allocation problem
// Made the constraint wTa <= w_cap*(random_weight)
std::set<State> expand_single_heli_stochastic(const State &curr_state, const ProblemData &problem, int num_samples, std::mt19937 &gen) {
    std::set<State> successors = std::set<State>();

    double values[3], weights[3];
    for (int i = 0; i < 3; ++i) {
        values[i] = problem.packages[i].value;
        weights[i] = problem.packages[i].weight;
    }

    // Store the villages that need more supplies
    std::vector<int> villages_left = std::vector<int>();
    for (size_t v = 0; v < problem.villages.size(); ++v) {
        if (curr_state.villageStates[v].help_needed) { villages_left.push_back(v); }
    }

    // Iterate through each helicopter
    for (size_t i = 0; i < problem.helicopters.size(); ++i) {
        HelicopterPlan heli_state = curr_state.heliStates[i];
        Helicopter heli = problem.helicopters[heli_state.helicopter_id-1];
        Point home_city = problem.cities[heli.home_city_id-1];

        for (int v: villages_left) {
            // Check if heli can travel to the village and come back
            double travel_dist = 2 * distance(home_city, problem.villages[v].coords);
            if ((travel_dist < (heli.distance_capacity + EPSILON)) && (travel_dist < (heli_state.d_max_left + EPSILON))) {
                // Choose num_samples-1 random numbers in range 0.2 to 0.8
                // And multiply with helicopter weight capacity to add stochasticity
                // Add weight 1.0
                std::uniform_real_distribution<double> dist(0.2, 0.8);

                std::vector<double> random_weights(num_samples);
                for (size_t rs = 0; rs < num_samples-1; ++rs) {
                    // VLA are Clang extension
                    random_weights[rs] = dist(gen);
                }
                random_weights[num_samples-1] = 1.0;

                for (double r_wt: random_weights) {
                    // Duplicate parent state, add a trip and then modify the village's state

                    State child_state = curr_state;

                    int dry_rec = child_state.villageStates[v].dry_food_rec;
                    int wet_rec = child_state.villageStates[v].wet_food_rec;
                    int other_rec = child_state.villageStates[v].other_food_rec;

                    int meal_req = (9 * problem.villages[v].population) - dry_rec - wet_rec;
                    int other_req = problem.villages[v].population - other_rec;

                    // Allocate supplies
                    std::pair<Point3d, double> allocation = solve_lp(weights, values, meal_req, other_req, heli.weight_capacity * r_wt);

                    // If no allocation, then skip
                    if ((allocation.first.x < EPSILON) && (allocation.first.y < EPSILON) && (allocation.first.z < EPSILON)) { continue; }

                    // Initialize a new trip
                    /** TODO: Check the order of dry, perishable, other*/
                    Trip t;
                    t.dry_food_pickup = allocation.first.x;
                    t.perishable_food_pickup = allocation.first.y;
                    t.other_supplies_pickup = allocation.first.z;
                    Drop d = {v+1, (int)allocation.first.x, (int)allocation.first.y, (int)allocation.first.z};
                    t.drops = vector<Drop>(1, d);
                    child_state.heliStates[i].trips.push_back(t);
                    child_state.heliStates[i].d_max_left -= travel_dist;
                    child_state.heliStates[i].in_trip = false;

                    // Modify the village's state
                    child_state.villageStates[v].dry_food_rec += allocation.first.x;
                    child_state.villageStates[v].wet_food_rec += allocation.first.y;
                    child_state.villageStates[v].other_food_rec += allocation.first.z;

                    if (child_state.villageStates[v].dry_food_rec + child_state.villageStates[v].wet_food_rec >= (9 * problem.villages[v].population - EPSILON) &&
                        child_state.villageStates[v].other_food_rec >= (problem.villages[v].population - EPSILON)) {
                        child_state.villageStates[v].help_needed = false;
                    }

                    child_state.g_cost = g(v, heli.home_city_id-1, heli_state.helicopter_id-1, problem, curr_state, r_wt);
                    // child_state.h_cost = h(heli_state.helicopter_id-1, v, problem, curr_state);
                    child_state.h_cost = 0;

                    successors.insert(child_state);
                }
            }
        }
    }

    return successors;
}

// Work in progress
// Enumerate all the possible situations in the conditions
std::set<State> expand(const State &curr_state, const ProblemData &problem, int num_samples, std::mt19937 &gen) {
    std::set<State> successors;

    double values[3], weights[3];
    for (int i = 0; i < 3; ++i) {
        values[i] = problem.packages[i].value;
        weights[i] = problem.packages[i].weight;
    }

    // Store the villages that need more supplies
    std::vector<int> villages_left = std::vector<int>();
    for (size_t v = 0; v < problem.villages.size(); ++v) {
        if (curr_state.villageStates[v].help_needed) { villages_left.push_back(v); }
    }

    // Iterate through each helicopter to find possible next moves
    for (size_t i = 0; i < curr_state.heliStates.size(); ++i) {
        const HelicopterPlan &heli_state = curr_state.heliStates[i];
        const Helicopter &heli = problem.helicopters[heli_state.helicopter_id-1];
        int curr_village_id = -1;
        bool at_home = false;   // If at home, or in a trip
        
        // Find the current location of the helicopter
        Point current_pos;
        Point home_city = problem.cities[heli.home_city_id - 1];
        if (!heli_state.in_trip) {
            // Condition that no trip is started or a trip has started but is empty
            current_pos = home_city;
            at_home = true;
        } else {
            int curr_village_id = heli_state.trips.back().drops.back().village_id;
            current_pos = problem.villages[curr_village_id - 1].coords;
        }

        if (!at_home) {
            State new_state = curr_state;

            new_state.heliStates[i].in_trip = false;
            new_state.heliStates[i].d_max_left -= distance(current_pos, home_city);
            new_state.g_cost = g(problem, curr_state, i, current_pos, home_city, 0.0, at_home);

            if (new_state.heliStates[i].d_max_left > -EPSILON) { successors.insert(new_state); }
        }

        for (auto v: villages_left) {
            if (v+1 == curr_village_id) { continue; }

            // Check for feasibility
            double travel_dist = distance(current_pos, problem.villages[v].coords) + distance(problem.villages[v].coords, home_city);
            if ((travel_dist < (heli.distance_capacity + EPSILON)) && (travel_dist < (heli_state.d_max_left + EPSILON))) {
                // Choose num_samples-1 random numbers in range 0.2 to 0.8
                // And multiply with helicopter weight capacity to add stochasticity
                std::uniform_real_distribution<double> dist(0.2, 0.8);

                std::vector<double> random_weights(num_samples);
                for (size_t rs = 0; rs < num_samples-1; ++rs) {
                    // VLA are Clang extension
                    random_weights[rs] = dist(gen);
                }
                random_weights[num_samples-1] = 1.0;

                for (double r_wt: random_weights) {
                    // Duplicate parent state, add a trip and then modify the village's state

                    State child_state = curr_state;

                    int dry_rec = child_state.villageStates[v].dry_food_rec;
                    int wet_rec = child_state.villageStates[v].wet_food_rec;
                    int other_rec = child_state.villageStates[v].other_food_rec;

                    int meal_req = (9 * problem.villages[v].population) - dry_rec - wet_rec;
                    int other_req = problem.villages[v].population - other_rec;

                    double w_cap_left = heli.weight_capacity;
                    if (!at_home) { w_cap_left = heli_state.trips.back().w_cap_left; }

                    // Allocate supplies
                    std::pair<Point3d, double> allocation = solve_lp(weights, values, meal_req, other_req, w_cap_left * r_wt);

                    // If no allocation, then skip
                    if ((allocation.first.x < EPSILON) && (allocation.first.y < EPSILON) && (allocation.first.z < EPSILON)) { continue; }

                    /** TODO: Check the order of dry, perishable, other*/
                    // Make a drop
                    Drop d = {v+1, (int)allocation.first.x, (int)allocation.first.y, (int)allocation.first.z};

                    double weight_supplies = (weights[0] * d.dry_food) + (weights[1] * d.perishable_food) + (weights[2] * d.other_supplies);
                    if (weight_supplies > w_cap_left) { continue; }

                    if (at_home) {
                        Trip t;
                        t.dry_food_pickup = allocation.first.x;
                        t.perishable_food_pickup = allocation.first.y;
                        t.other_supplies_pickup = allocation.first.z;
                        t.w_cap_left = w_cap_left - weight_supplies;
                        t.drops = vector<Drop>(1, d);
                        child_state.heliStates[i].trips.push_back(t);
                        child_state.heliStates[i].in_trip = true;
                    } else {
                        child_state.heliStates[i].trips.back().drops.push_back(d);
                        child_state.heliStates[i].trips.back().w_cap_left -= weight_supplies;
                    }

                    child_state.heliStates[i].d_max_left -= travel_dist;

                    // Modify the village's state
                    child_state.villageStates[v].dry_food_rec += allocation.first.x;
                    child_state.villageStates[v].wet_food_rec += allocation.first.y;
                    child_state.villageStates[v].other_food_rec += allocation.first.z;

                    if (child_state.villageStates[v].dry_food_rec + child_state.villageStates[v].wet_food_rec >= (9 * problem.villages[v].population - EPSILON) &&
                        child_state.villageStates[v].other_food_rec >= (problem.villages[v].population - EPSILON)) {
                        child_state.villageStates[v].help_needed = false;
                    }

                    child_state.g_cost = g(problem, curr_state, i, current_pos, problem.villages[v].coords, allocation.second, at_home);
                    // child_state.h_cost = h(heli_state.helicopter_id-1, v, problem, curr_state);
                    child_state.h_cost = 0;

                    successors.insert(child_state);
                }
            }
        }
    }
    return successors;
}