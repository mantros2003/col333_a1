#include <iostream>
#include <vector>
#include <queue>
#include <set>
#include <random>

#include "helper.h"

/**
 * Calculates the total distance traveled by a single helicopter in all its trips
 * 
 * Each trip starts at the home city, visits some villages, and may return to the home city
 * @param heli_state The state of a single helicopter.
 * @return The total distance traveled in kilometers.
 */
inline double calculate_total_heli_distance(const HeliState& heli_state, const ProblemData &problem) {
    double total_distance = 0.0;

    // Iterate through each trip undertaken by the helicopter
    for (const Trip &trip : heli_state.trips) { total_distance += trip.distance_covered; }

    return total_distance;
}

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
            for (int i = 0; i < trip.villages.size(); i++) {
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
 * Check if the village's needs are fulfilled
 * @param s The state
 * @param v The village id
 */
// Remove this function, use the bool help_needed
// bool is_village_satisfied(const State &s, int v) {
//     VillageState state = s.villageStates[v-1];
//     int village_pop = state.vill.population;

//     return ((state.dry_food_rec + state.wet_food_rec) >= 9 * village_pop) & (state.other_food_rec >= village_pop);
// }

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


// Not working on this implemntation
/** TODO: Move the function to solver.cpp */
State expand(State &s, const ProblemData &problem) {
    int num_helicopters = s.heliStates.size();
    priority_queue<int, vector<int>, greater<int>> min_pq;

    for (int i = 0; i < num_helicopters; ++i) {
        
    }
}

std::vector<Node> expand_single_heli(const Node &parent_node, const ProblemData &problem, double (*g)(State), double (*h)(State)) {
    std::vector<Node> successors;
    const State &parent_state = parent_node.state;

    double values[3], weights[3];
    for (int i = 0; i < 3; ++i) {
        values[i] = problem.packages[i].value;
        weights[i] = problem.packages[i].weight;
    }

    // Store the villages that need more supplies
    std::vector<int> villages_left;
    for (size_t v = 0; v < problem.villages.size(); ++v) {
        if (! is_village_satisfied(parent_state, problem.villages[v].id)) { villages_left.push_back(v); }
    }

    // Iterate through each helicopter
    for (size_t i = 0; i < problem.helicopters.size(); ++i) {
        HeliState heli_state = parent_state.heliStates[i];
        double d_travelled = calculate_total_heli_distance(heli_state, problem);
        for (int v: villages_left) {
            // Check if heli can travel to the village and come back
            double travel_dist = 2*distance(problem.cities[heli_state.heli.id-1], problem.villages[v].coords);
            if (travel_dist <= heli_state.heli.distance_capacity && travel_dist <= problem.d_max - d_travelled) {
                // Allocate supplies
                std::pair<Point3d, double> allocation = solve_lp(values, weights, problem.villages[v].population, heli_state.heli.weight_capacity);

                // Duplicate parent state, add a trip and then modify the village's state
                State child_state = parent_state;

                // Initialize a new trip
                Trip t = {{v}, true, travel_dist, {allocation.first.x, allocation.first.y, allocation.first.z}};
                child_state.heliStates[i].trips.push_back(t);

                // Modify the village's state
                child_state.villageStates[v].dry_food_rec += allocation.first.x;
                child_state.villageStates[v].wet_food_rec += allocation.first.y;
                child_state.villageStates[v].other_food_rec += allocation.first.z;

                successors.push_back({child_state, g(child_state), h(child_state)});
            }
        }
    }

    return successors;
}

// Same as the above function, just added stochasticity to the resource allocation problem
// Made the constraint wTa <= w_cap*(random_weight)
std::vector<Node> expand_single_heli_stochastic(const Node &parent_node, const ProblemData &problem, double (*g)(State), double (*h)(State), int num_samples) {
    std::vector<Node> successors;
    const State &parent_state = parent_node.state;

    double values[3], weights[3];
    for (int i = 0; i < 3; ++i) {
        values[i] = problem.packages[i].value;
        weights[i] = problem.packages[i].weight;
    }

    // Store the villages that need more supplies
    std::vector<int> villages_left;
    for (size_t v = 0; v < problem.villages.size(); ++v) {
        if (! is_village_satisfied(parent_state, problem.villages[v].id)) { villages_left.push_back(v); }
    }

    // Iterate through each helicopter
    for (size_t i = 0; i < problem.helicopters.size(); ++i) {
        HeliState heli_state = parent_state.heliStates[i];
        double d_travelled = calculate_total_heli_distance(heli_state, problem);
        for (int v: villages_left) {
            // Check if heli can travel to the village and come back
            double travel_dist = 2*distance(problem.cities[heli_state.heli.id-1], problem.villages[v].coords);
            if (travel_dist <= heli_state.heli.distance_capacity && travel_dist <= problem.d_max - d_travelled) {
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
                    std::pair<Point3d, double> allocation = solve_lp(values, weights, problem.villages[v].population, heli_state.heli.weight_capacity * r_wt);

                    // Duplicate parent state, add a trip and then modify the village's state
                    State child_state = parent_state;

                    // Initialize a new trip
                    Trip t = {{v}, true, travel_dist, {allocation.first.x, allocation.first.y, allocation.first.z}};
                    child_state.heliStates[i].trips.push_back(t);

                    // Modify the village's state
                    child_state.villageStates[v].dry_food_rec += allocation.first.x;
                    child_state.villageStates[v].wet_food_rec += allocation.first.y;
                    child_state.villageStates[v].other_food_rec += allocation.first.z;

                    successors.push_back({child_state, g(child_state), h(child_state)});
                }
            }
        }
    }

    return successors;
}

// Work in progress
// Enumerate all the possible situations in the conditions
std::vector<Node> expand(const Node &parent_node, const ProblemData &problem, double (*g)(State), double (*h)(State)) {
    std::vector<Node> successors;
    const State& parent_state = parent_node.state;

    // Iterate through each helicopter to find possible next moves
    for (size_t i = 0; i < parent_state.heliStates.size(); ++i) {
        const HeliState &heli_state = parent_state.heliStates[i];
        const Helicopter &heli = heli_state.heli;
        bool at_home = false;   // If at home, or in a trip
        
        // Find the current location of the helicopter
        Point current_pos;
        if (heli_state.trips.empty() || heli_state.trips.back().villages.back() == -1) {
            // Condition that no trip is started or a trip has started but is empty
            current_pos = problem.cities[heli.home_city_id - 1];
            at_home = true;
        } else {
            int last_village_id = heli_state.trips.back().villages.back();
            current_pos = problem.villages[last_village_id - 1].coords;
        }

        // If helicopter at home city, then look for a trip
        if (at_home) {
            for (const Village &v: problem.villages) {
                if (! is_village_satisfied(parent_state, v.id)) {
                    const VillageState& village_sate = parent_state.villageStates[v.id-1];

                    // Additional requirement of the village
                    int meals_needed = (v.population * 9) - (village_sate.wet_food_rec + village_sate.dry_food_rec);
                    int other_needed = v.population - (village_sate.other_food_rec);

                    // int max_meals = std::max(meals_needed, heli.weight_capacity);
                    int max_other = std::max(other_needed * problem.packages[2].weight, heli.weight_capacity);
                }
            }
        }

        // Action 1: Start a new trip to an unvisited village
        for (const auto &vs : parent_state.villageStates) {
            // Check if the village has remaining needs and can be reached by a new trip
            // REDO and CHECK this section
            if (! is_village_satisfied(parent_state, vs.vill.id)) {
                
                // Create a copy of the parent state
                State new_state = parent_state;
                
                // Calculate minimum necessary packages for the trip
                int meals_needed = vs.vill.population * 9;
                int other_needed = vs.vill.population;
                
                // Greedily prioritize perishable food for value
                int perishable_deliver = std::min(meals_needed, 10000);
                int dry_deliver = std::min(meals_needed - perishable_deliver, 10000);
                int other_deliver = std::min(other_needed, 10000);

                double trip_weight = perishable_deliver * problem.packages[0].weight +
                                        dry_deliver * problem.packages[1].weight +
                                        other_deliver * problem.packages[2].weight;

                double trip_dist = distance(problem.cities[heli.home_city_id-1], vs.vill.coords) +
                                   distance(vs.vill.coords, problem.cities[heli.home_city_id-1]);

                // Check feasibility of the new trip
                if (trip_weight <= heli.weight_capacity && trip_dist <= heli.distance_capacity && (calculate_total_heli_distance(heli_state, problem) + trip_dist) <= problem.d_max) {
                    
                    // Update the state
                    // new_state.heliStates[i].trips.push_back({vs.vill.id});
                    // new_state.heliStates[i].heli.total_dist_traveled += trip_dist;
                    new_state.heliStates[i].dry_food_left += dry_deliver;
                    new_state.heliStates[i].wet_food_left += perishable_deliver;
                    new_state.heliStates[i].other_food_left += other_deliver;
                    new_state.villageStates[vs.vill.id - 1].dry_food_rec += dry_deliver;
                    new_state.villageStates[vs.vill.id - 1].wet_food_rec += perishable_deliver;
                    new_state.villageStates[vs.vill.id - 1].other_food_rec += other_deliver;
                    // new_state.villageStates[vs.vill.id - 1].vill.remaining_people = 0;

                    // Create and add the new node to the successors list
                    Node new_node;
                    new_node.state = new_state;
                    new_node.g = g(new_state);
                    new_node.h = h(new_state);
                    successors.push_back(new_node);
                }
            }
        }
    }

    return successors;
}