#include <iostream>
#include <chrono>
#include <limits>
#include <random>
#include <stack>
#include <iterator>
#include "solver.h"
#include "expand.h"
#include "lp_solver.h"
// #include <cmath>

using namespace std;
// You can add any helper functions or classes you need here.

/**
 * @brief The main function to implement your search/optimization algorithm.
 * * This is a placeholder implementation. It creates a simple, likely invalid,
 * plan to demonstrate how to build the Solution object. 
 * * TODO: REPLACE THIS ENTIRE FUNCTION WITH YOUR ALGORITHM.
 */

Solution solve_heuristic(const ProblemData& problem) {
    cout << "Starting solver..." << endl;

    // This leaves time for file I/O and other wrap-up tasks.
    const auto safety_margin = chrono::milliseconds(200);
    auto allowed_duration = chrono::duration<double>(problem.time_limit_minutes * 60.0);
        
    // Calculate the absolute deadline by adding the duration to the current time.
    auto start_time = chrono::steady_clock::now();
    auto deadline = start_time + chrono::duration_cast<chrono::steady_clock::duration>(allowed_duration);

    Solution solution;
    long long states_explored = 0;

    // For random number generation
    std::random_device rd;
    std::mt19937 gen(rd());

    // Initialize the start state
    State root_state, current_state;
    vector<HelicopterPlan> h_plan(problem.helicopters.size());
    vector<V_state> village_state(problem.villages.size(), {true, 0, 0, 0});
    for (int i = 0; i < h_plan.size(); i++){
        h_plan[i].helicopter_id = i+1;
        h_plan[i].trips = vector<Trip>();
        h_plan[i].d_max_left = problem.d_max;
    }
    root_state = {0, 0, h_plan, village_state};
    current_state = root_state;

    set<State> frontier = set<State>();
    frontier.insert(current_state);

    double best_f = -1 * numeric_limits<double>::infinity();
    State best_state = root_state;

    while(!frontier.empty()){
        // cout << "Size of frontier: " << frontier.size() << endl;
        states_explored++;

        if (states_explored % 1000 == 0) {
            if (chrono::steady_clock::now() > deadline - safety_margin) {
                cout << "\nTime limit approaching. Terminating search." << endl;
                break;
            }
        }
        if (states_explored % 10000 == 0) { cout << frontier.size() << endl; }
        current_state = *frontier.begin();
        if (current_state.g_cost + current_state.h_cost > best_f) {
            best_state = current_state;
            best_f = current_state.g_cost + current_state.h_cost;
        }
        // cout << current_state.g_cost + current_state.h_cost << endl;
        frontier.erase(frontier.begin());
        frontier.merge(expand_single_heli_stochastic(current_state, problem, 1, gen));
    }
    solution = best_state.heliStates;
    
    // --- END OF PLACEHOLDER LOGIC ---

    cout << "Solver finished." << endl;
    cout << states_explored << " states explored!" << endl;
    cout << "Best g value: " << best_f << endl;
    cout << "Best states f value: " << (best_state.g_cost + best_state.h_cost) << endl;
    return solution;
}

Solution solve_dfs(const ProblemData& problem) {
    cout << "Starting solver..." << endl;

    // This leaves time for file I/O and other wrap-up tasks.
    const auto safety_margin = chrono::milliseconds(200);
    auto allowed_duration = chrono::milliseconds(long(problem.time_limit_minutes * 60 * 1000));
        
    // Calculate the absolute deadline by adding the duration to the current time.
    auto start_time = chrono::steady_clock::now();
    auto deadline = start_time + allowed_duration;

    Solution solution;
    long long states_explored = 0;

    // For random number generation
    std::random_device rd;
    std::mt19937 gen(rd());

    // Initialize the start state
    State root_state, current_state;
    vector<HelicopterPlan> h_plan(problem.helicopters.size());
    vector<V_state> village_state(problem.villages.size(), {true, 0, 0, 0});
    for (int i = 0; i < h_plan.size(); i++){
        h_plan[i].helicopter_id = i+1;
        h_plan[i].trips = vector<Trip>();
        h_plan[i].d_max_left = problem.d_max;
    }
    root_state = {0, 0, h_plan, village_state};
    current_state = root_state;

    stack<State> frontier = stack<State>();
    frontier.push(current_state);

    double best_f = 0;
    State best_state = root_state;

    while(!frontier.empty()){
        // cout << "Size of frontier: " << frontier.size() << endl;
        states_explored++;

        if (states_explored % 1000 == 0) {
            if (chrono::steady_clock::now() > deadline - safety_margin) {
                cout << "\nTime limit approaching. Terminating search." << endl;
                break;
            }
        }
        if (states_explored % 2 == 0) { cout << frontier.size() << endl; }

        current_state = frontier.top();
        frontier.pop();
        if (current_state.g_cost + current_state.h_cost > best_f) {
            best_state = current_state;
            best_f = current_state.g_cost + current_state.h_cost;
        }

        // cout << current_state.g_cost + current_state.h_cost << endl;
        set<State> new_nodes = expand_single_heli_stochastic(current_state, problem, 2, gen);
        for (auto it = new_nodes.rbegin(); it != new_nodes.rend(); ++it) {
            frontier.push(*it);
        }
    }
    solution = best_state.heliStates;

    cout << "Solver finished." << endl;
    cout << states_explored << " states explored!" << endl;
    cout << "Best g value: " << best_f << endl;
    cout << "Best states f value: " << (best_state.g_cost + best_state.h_cost) << endl;
    return solution;
}

Solution solve_beam_bfs(const ProblemData& problem) {
    cout << "Starting solver..." << endl;

    constexpr long BEAM_WIDTH = 100000;

    // This leaves time for file I/O and other wrap-up tasks.
    const auto safety_margin = chrono::milliseconds(200);
    auto allowed_duration = chrono::milliseconds(long(problem.time_limit_minutes * 60 * 1000));
        
    // Calculate the absolute deadline by adding the duration to the current time.
    auto start_time = chrono::steady_clock::now();
    auto deadline = start_time + allowed_duration;

    Solution solution;
    long long states_explored = 0;

    // For random number generation
    std::random_device rd;
    std::mt19937 gen(rd());

    // Initialize the start state
    State root_state, current_state;
    vector<HelicopterPlan> h_plan(problem.helicopters.size());
    vector<V_state> village_state(problem.villages.size(), {true, 0, 0, 0});
    for (int i = 0; i < h_plan.size(); i++){
        h_plan[i].helicopter_id = i+1;
        h_plan[i].trips = vector<Trip>();
        h_plan[i].d_max_left = problem.d_max;
    }
    root_state = {0, 0, h_plan, village_state};
    current_state = root_state;

    set<State> frontier = set<State>();
    frontier.insert(current_state);

    double best_f = 0;
    State best_state = root_state;

    while(!frontier.empty()){
        set<State> new_frontier = set<State>();
        for (auto current_state: frontier) {

            // cout << "Size of frontier: " << frontier.size() << endl;
            states_explored++;

            if (states_explored % 1000 == 0) {
                if (chrono::steady_clock::now() > deadline - safety_margin) {
                    cout << "\nTime limit approaching. Terminating search." << endl;
                    break;
                }
            }
            if (states_explored % 10000 == 0) { cout << frontier.size() << endl; }

            if (current_state.g_cost + current_state.h_cost > best_f) {
                best_state = current_state;
                best_f = current_state.g_cost + current_state.h_cost;
            }

            // cout << current_state.g_cost + current_state.h_cost << endl;
            {
                set<State> sucessors = expand_single_heli_stochastic(current_state, problem, 1, gen);
                new_frontier.merge(sucessors);
            }
            if (new_frontier.size() > BEAM_WIDTH) {
                set<State>::iterator it = new_frontier.begin();
                std::advance(it, BEAM_WIDTH);
                new_frontier.erase(it, new_frontier.end());
            }
        }
        frontier = new_frontier;
        if (chrono::steady_clock::now() > deadline - safety_margin) {
            cout << "\nTime limit approaching. Terminating search." << endl;
            break;
        }
    }
    solution = best_state.heliStates;

    cout << "Solver finished." << endl;
    cout << states_explored << " states explored!" << endl;
    cout << "Best g value: " << best_f << endl;
    cout << "Best states f value: " << (best_state.g_cost + best_state.h_cost) << endl;
    return solution;
}

Solution solve_beam_dfs(const ProblemData& problem) {
    cout << "Starting solver..." << endl;

    constexpr long BEAM_WIDTH = 100000;

    // This leaves time for file I/O and other wrap-up tasks.
    const auto safety_margin = chrono::milliseconds(200);
    auto allowed_duration = chrono::milliseconds(long(problem.time_limit_minutes * 60 * 1000));
        
    // Calculate the absolute deadline by adding the duration to the current time.
    auto start_time = chrono::steady_clock::now();
    auto deadline = start_time + allowed_duration;

    Solution solution;
    long long states_explored = 0;

    // For random number generation
    std::random_device rd;
    std::mt19937 gen(rd());

    // Initialize the start state
    State root_state, current_state;
    vector<HelicopterPlan> h_plan(problem.helicopters.size());
    vector<V_state> village_state(problem.villages.size(), {true, 0, 0, 0});
    for (int i = 0; i < h_plan.size(); i++){
        h_plan[i].helicopter_id = i+1;
        h_plan[i].trips = vector<Trip>();
        h_plan[i].d_max_left = problem.d_max;
    }
    root_state = {0, 0, h_plan, village_state};
    current_state = root_state;

    set<State> frontier = set<State>();
    frontier.insert(current_state);

    double best_f = 0;
    State best_state = root_state;

    while(!frontier.empty()){
        // cout << "Size of frontier: " << frontier.size() << endl;
        states_explored++;

        if (states_explored % 1000 == 0) {
            if (chrono::steady_clock::now() > deadline - safety_margin) {
                cout << "\nTime limit approaching. Terminating search." << endl;
                break;
            }
        }
        if (states_explored % 10000 == 0) { cout << frontier.size() << endl; }

        current_state = *frontier.begin();
        frontier.erase(frontier.begin());
        if (current_state.g_cost + current_state.h_cost > best_f) {
            best_state = current_state;
            best_f = current_state.g_cost + current_state.h_cost;
        }

        // cout << current_state.g_cost + current_state.h_cost << endl;
        set<State> sucessors = expand_single_heli_stochastic(current_state, problem, 1, gen);
        frontier.merge(sucessors);
    
        if (frontier.size() > BEAM_WIDTH) {
            set<State>::iterator it = frontier.begin();
            std::advance(it, BEAM_WIDTH);
            frontier.erase(it, frontier.end());
        }
    }

    solution = best_state.heliStates;

    cout << "Solver finished." << endl;
    cout << states_explored << " states explored!" << endl;
    cout << "Best g value: " << best_f << endl;
    cout << "Best states f value: " << (best_state.g_cost + best_state.h_cost) << endl;
    return solution;
}