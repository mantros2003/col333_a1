#include <iostream>
#include <vector>
#include <queue>
#include <set>
#include <random>
#include "cost_function.h"
#include "structures.h"
#include "helper.h"

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

        Point home_city = problem.cities[heli.home_city_id-1];
        // Calculate the total distance covered by helicopter over all the trips
        double total_trip_dist = calculate_total_heli_distance(s.heliStates[heli_state.helicopter_id-1], problem);

        value -= (total_trip_dist * heli.alpha);
    }

    double w_d, w_w, w_o;   // Weights
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
        Point home_city = problem.cities[problem.helicopters[heli_state.helicopter_id-1].home_city_id-1];
        double d_travelled = calculate_total_heli_distance(heli_state, problem);

        for (int v: villages_left) {
            // Check if heli can travel to the village and come back
            double travel_dist = 2*distance(home_city, problem.villages[v].coords);
            if (travel_dist <= heli.distance_capacity && travel_dist <= problem.d_max - d_travelled) {
                // Allocate supplies
                std::pair<Point3d, double> allocation = solve_lp(problem, curr_state, v, heli.weight_capacity);
                
                // Duplicate parent state, add a trip and then modify the village's state
                State child_state = curr_state;
                
                // Initialize a new trip
                /** TODO: Check the order of dry, perishable, other*/
                Trip t;
                t.dry_food_pickup = allocation.first.x;
                t.perishable_food_pickup = allocation.first.y;
                t.other_supplies_pickup = allocation.first.z;
                Drop d = {v+1, (int)allocation.first.x, (int)allocation.first.y, (int)allocation.first.z};
                t.drops = vector<Drop>(1, d);
                child_state.heliStates[i].trips.push_back(t);
                
                // Modify the village's state
                child_state.villageStates[v].dry_food_rec += allocation.first.x;
                child_state.villageStates[v].wet_food_rec += allocation.first.y;
                child_state.villageStates[v].other_food_rec += allocation.first.z;
                // condition for village's needs
                if (child_state.villageStates[v].dry_food_rec + child_state.villageStates[v].wet_food_rec > 0.9 * problem.villages[v].population){
                    child_state.villageStates[v].help_needed = false;
                }
                child_state.g_cost = g(v,heli.home_city_id-1,heli_state.helicopter_id-1, problem, curr_state);
                child_state.h_cost = h(heli_state.helicopter_id-1, v, problem, curr_state);
                successors.insert(child_state);
            }
        }
    }
    return successors;
}

// Same as the above function, just added stochasticity to the resource allocation problem
// Made the constraint wTa <= w_cap*(random_weight)
std::vector<State> expand_single_heli_stochastic(const State &curr_state, const ProblemData &problem, double (*g)(State), double (*h)(State), int num_samples) {
    std::vector<State> successors;

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
        Point home_city = problem.cities[problem.helicopters[heli_state.helicopter_id-1].home_city_id-1];
        double d_travelled = calculate_total_heli_distance(heli_state, problem);

        for (int v: villages_left) {
            // Check if heli can travel to the village and come back
            double travel_dist = 2*distance(home_city, problem.villages[v].coords);
            if (travel_dist <= heli.distance_capacity && travel_dist <= problem.d_max - d_travelled) {
                // Choose num_samples-1 random numbers in range 0.2 to 0.8
                // And multiply with helicopter weight capacity to add stochasticity
                // Add weight 1.0
                std::random_device rd;
                std::mt19937 gen(rd());  // random engine
                std::uniform_real_distribution<double> dist(0.2, 0.8);

                double random_weights[num_samples];
                for (size_t rs = 0; rs < num_samples-1; ++rs) {
                    random_weights[rs] = dist(gen);
                }
                random_weights[num_samples-1] = 1.0;
                for (double r_wt: random_weights) {
                    // Allocate supplies
                std::pair<Point3d, double> allocation = solve_lp(problem, curr_state, problem.villages[v].population, heli.weight_capacity);

                // Duplicate parent state, add a trip and then modify the village's state
                State child_state = curr_state;

                // Initialize a new trip
                /** TODO: Check the order of dry, perishable, other*/
                Trip t;
                t.dry_food_pickup = allocation.first.x;
                t.perishable_food_pickup = allocation.first.y;
                t.other_supplies_pickup = allocation.first.z;
                Drop d = {v+1, (int)allocation.first.x, (int)allocation.first.y, (int)allocation.first.z};
                t.drops = vector<Drop>(1, d);
                child_state.heliStates[i].trips.push_back(t);

                // Modify the village's state
                child_state.villageStates[v].dry_food_rec += allocation.first.x;
                child_state.villageStates[v].wet_food_rec += allocation.first.y;
                child_state.villageStates[v].other_food_rec += allocation.first.z;

                successors.push_back(child_state);
                }
            }
        }
    }

    return successors;
}

// Work in progress
// Enumerate all the possible situations in the conditions
std::vector<State> expand(const State &curr_state, const ProblemData &problem, double (*g)(State), double (*h)(State)) {
    std::vector<State> successors;

    // Iterate through each helicopter to find possible next moves
    for (size_t i = 0; i < curr_state.heliStates.size(); ++i) {
        const HelicopterPlan &heli_state = curr_state.heliStates[i];
        const Helicopter &heli = problem.helicopters[heli_state.helicopter_id-1];
        bool at_home = false;   // If at home, or in a trip
        
        // Find the current location of the helicopter
        // Not implemented "-1" functionality
        Point current_pos;
        if (heli_state.trips.empty() || heli_state.trips.back().drops.back().village_id == -1) {
            // Condition that no trip is started or a trip has started but is empty
            current_pos = problem.cities[heli.home_city_id - 1];
            at_home = true;
        } else {
            int last_village_id = heli_state.trips.back().drops.back().village_id;
            current_pos = problem.villages[last_village_id - 1].coords;
        }

        // If helicopter at home city, then look for a trip
        if (at_home) {
            for (const Village &v: problem.villages) {
                if (! curr_state.villageStates[v.id-1].help_needed) {
                    const V_state& village_sate = curr_state.villageStates[v.id-1];

                    // Additional requirement of the village
                    int meals_needed = (v.population * 9) - (village_sate.wet_food_rec + village_sate.dry_food_rec);
                    int other_needed = v.population - (village_sate.other_food_rec);

                    // int max_meals = std::max(meals_needed, heli.weight_capacity);
                    int max_other = std::max(other_needed * problem.packages[2].weight, heli.weight_capacity);
                }
            }
        }

        // Action 1: Start a new trip to an unvisited village
        for (int vs = 0; vs < problem.villages.size(); ++vs) {
            // Check if the village has remaining needs and can be reached by a new trip
            // REDO and CHECK this section
            if (! curr_state.villageStates[vs].help_needed) {
                Village v = problem.villages[vs];
                // Create a copy of the parent state
                State new_state = curr_state;
                
                // Calculate minimum necessary packages for the trip
                int meals_needed = v.population * 9;
                int other_needed = v.population;
                
                // Greedily prioritize perishable food for value
                int perishable_deliver = std::min(meals_needed, 10000);
                int dry_deliver = std::min(meals_needed - perishable_deliver, 10000);
                int other_deliver = std::min(other_needed, 10000);

                double trip_weight = perishable_deliver * problem.packages[0].weight +
                                        dry_deliver * problem.packages[1].weight +
                                        other_deliver * problem.packages[2].weight;

                double trip_dist = distance(problem.cities[heli.home_city_id-1], v.coords) +
                                   distance(v.coords, problem.cities[heli.home_city_id-1]);

                // Check feasibility of the new trip
                if (trip_weight <= heli.weight_capacity && trip_dist <= heli.distance_capacity && (calculate_total_heli_distance(heli_state, problem) + trip_dist) <= problem.d_max) {
                    
                    // Update the state
                    // new_state.heliStates[i].trips.push_back({vs.vill.id});
                    // new_state.heliStates[i].heli.total_dist_traveled += trip_dist;
                    new_state.villageStates[vs].dry_food_rec += dry_deliver;
                    new_state.villageStates[vs].wet_food_rec += perishable_deliver;
                    new_state.villageStates[vs].other_food_rec += other_deliver;
                    // new_state.villageStates[vs.vill.id - 1].vill.remaining_people = 0;

                    // Create and add the new node to the successors list
                    new_state.g_cost = g(new_state);
                    new_state.h_cost = h(new_state);
                    successors.push_back(new_state);
                }
            }
        }
    }

    return successors;
}