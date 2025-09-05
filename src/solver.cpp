#include "solver.h"
#include "helper.h"
#include "lp_solver.h"
#include <iostream>
#include "solver.h"
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



Solution solve(const ProblemData& problem) {
    cout << "Starting solver..." << endl;
    
    
    // --- START OF PLACEHOLDER LOGIC ---
    // This is a naive example: send each helicopter on one trip to the first village.
    // This will definitely violate constraints but shows the structure.
    
    // for (const auto& helicopter : problem.helicopters) {
        //     HelicopterPlan plan;
        //     plan.helicopter_id = helicopter.id;
        //     if (!problem.villages.empty()) {
            //         Trip trip;
            //         // Pickup 1 of each package type
            //         trip.dry_food_pickup = 1;
            //         trip.perishable_food_pickup = 1;
            //         trip.other_supplies_pickup = 1;
            //         // Drop them at the first village
            //         Drop drop;
            //         drop.village_id = problem.villages[0].id;
            //         drop.dry_food = 1;
            //         drop.perishable_food = 1;
            //         drop.other_supplies = 1;
            //         trip.drops.push_back(drop);
            //         plan.trips.push_back(trip);
            //     }
            //     solution.push_back(plan);
            // }
    
    // solution = vector<HelicopterPlan>
    Solution solution;
    int count = 0;
    // Initialize the start state
    State root_state, current_state;
    vector<HelicopterPlan> h_plan(problem.helicopters.size());
    cout<<"start_state : \n";
    for (const auto& plan : h_plan) {
        cout << plan.helicopter_id << " " << plan.trips.size() << "\n";
        for (const auto& trip : plan.trips) {
            cout << trip.dry_food_pickup << " " << trip.perishable_food_pickup << " " << trip.other_supplies_pickup << " " << trip.drops.size();
            for (const auto& drop : trip.drops) {
                cout<< " " << drop.village_id << " " << drop.dry_food << " " << drop.perishable_food << " " << drop.other_supplies;
}
            cout<< "\n";
        }
        cout<< -1 << "\n";
    }
    vector<V_state> village_state(problem.villages.size(), {true, 0, 0, 0});
    vector<Trip> temp_trip;
    for (int i = 0; i < h_plan.size(); i++){
        h_plan[i].helicopter_id = i+1;
        h_plan[i].trips = temp_trip;
    }
    root_state = {0,0, h_plan, village_state};
    current_state = root_state;
    set<State> frontier;
    frontier.insert(current_state);
    while(count < 100 && !frontier.empty()){
        cout<<"count: "<<count<<"\n";
        count++;
        frontier = expand_single_heli(current_state, problem);
        current_state = *frontier.begin();
    }
    solution = current_state.heliStates;
    
    // --- END OF PLACEHOLDER LOGIC ---

    cout << "Solver finished." << endl;
    return solution;
}