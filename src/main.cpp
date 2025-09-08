#include <iostream>
#include <string>
#include "structures.h"
#include "io_handler.h"
#include "solver.h"

using namespace std;

void verify(Solution &s, const ProblemData &p) {
    double score = 0.0;
    vector<vector<int>> supplies_dropped = vector<vector<int>>(p.villages.size(), {0, 0, 0});

    for (auto &plan: s) {
        Helicopter heli = p.helicopters[plan.helicopter_id-1];
        Point home_city = p.cities[heli.home_city_id-1];

        double F = heli.fixed_cost; double alpha = heli.alpha;
        double D_max = p.d_max;
        double heli_score = 0.0;
        for (int i = 0; i < plan.trips.size(); i++) {
            auto trip = plan.trips[i];
            double trip_dist = 0.0, trip_weight = 0.0;
            Point curr_pos = home_city;

            for (auto &drop: trip.drops) {
                trip_dist += distance(curr_pos, p.villages[drop.village_id-1].coords);

                supplies_dropped[drop.village_id-1][0] += drop.dry_food;
                supplies_dropped[drop.village_id-1][1] += drop.perishable_food;
                supplies_dropped[drop.village_id-1][2] += drop.other_supplies;

                trip_weight += ((p.packages[0].weight * drop.dry_food) + (p.packages[1].weight * drop.perishable_food) + (p.packages[2].weight * drop.other_supplies));

                curr_pos = p.villages[drop.village_id-1].coords;
            }

            trip_dist += distance(curr_pos, home_city);

            // Weight constraint
            if (trip_weight > heli.weight_capacity + 1e-9) {
                cout << "Weight limit exceeded for trip " << i << " of helicopter " << heli.id << endl;
            }

            // Distance constraint
            if (trip_dist > heli.distance_capacity + 1e-9) {
                cout << "Distance limit exceeded for trip " << i << " of helicopter " << heli.id << endl;
                cout << "Trip from city " << heli.home_city_id << " to village " << trip.drops[0].village_id << endl;
                cout << "Trip distance: " << trip_dist << " " << heli.distance_capacity << endl;
            }

            // Total distance constraint
            if (trip_dist > D_max + 1e-9) {
                cout << "D_max limit exceeded for trip " << i << " of helicopter " << heli.id << endl;
                cout << "Trip from city " << heli.home_city_id << " to village " << trip.drops[0].village_id << endl;
                cout << trip_dist << " " << D_max << endl;
            }

            D_max -= trip_dist;

            double trip_value = ((p.packages[0].value * trip.dry_food_pickup) + (p.packages[1].value * trip.perishable_food_pickup) + (p.packages[2].value * trip.other_supplies_pickup));
            double trip_score = trip_value - (F + (alpha * trip_dist));
            heli_score += trip_score;
            score += trip_score;
            cout << "Trip score: " << trip_score << " \tHeli Score: " << heli_score << " \tTotal Score: " << score << endl;
        }
    }
}

int main(int argc, char* argv[]) {
    if (argc != 3) {
        cerr << "Usage: " << argv[0] << " <input_filename> <output_filename>" << endl;
        return 1;
    }

    string input_filename = argv[1];
    string output_filename = argv[2];

    try {
        // 1. Read problem data from input file
        ProblemData problem = readInputData(input_filename);
        cout << "Successfully read input file: " << input_filename << endl;

        auto allowed_duration = chrono::milliseconds(long(problem.time_limit_minutes * 60 * 1000));
        auto start_time = chrono::steady_clock::now();
        auto deadline = start_time + allowed_duration;

        // 2. Solve the problem
        Solution solution = solve_beam_dfs(problem);

        cout << "solver exited!" << endl;

        verify(solution, problem);
        
        auto end_time = chrono::steady_clock::now();
        auto elapsed = chrono::duration_cast<chrono::milliseconds>(end_time - start_time);
        cout << "Solver completed in " << elapsed.count() / 1000.0 << " seconds." << endl;

        if (end_time > deadline) {
            cerr << "TimeLimitExceeded: Solver exceeded the time limit of " << problem.time_limit_minutes << " minutes." << endl;
            cout << "This instance will receive a score of 0." << endl;
            // return 1;
        }

        // 3. Write the solution to the output file
        writeOutputData(output_filename, solution);
        cout << "Successfully wrote solution to output file: " << output_filename << endl;

    } catch (const runtime_error& e) {
        cerr << "An error occurred: " << e.what() << endl;
        return 1;
    }

    return 0;
}