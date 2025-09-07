#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <stdexcept>
#include <iomanip>
#include <numeric>
#include <algorithm>

#include "structures.h"
#include "io_handler.h"

using namespace std;

/**
 * Robust verification and scoring.
 * Reads tokens directly from the output stream so newlines/spacing do not matter.
 */
double verifyAndCalculateScore(const string& input_file_path, const string& output_file_path) {
    ProblemData data = readInputData(input_file_path);

    ifstream outfile(output_file_path);
    if (!outfile.is_open()) {
        throw runtime_error("Error: Could not open output file " + output_file_path);
    }

    bool constraint_violated = false;

    // 1-based indexing for villages and helicopters, like the original
    vector<double> food_delivered(data.villages.size() + 1, 0.0);
    vector<double> other_delivered(data.villages.size() + 1, 0.0);
    vector<double> village_values(data.villages.size() + 1, 0.0);
    vector<double> helicopter_total_distances(data.helicopters.size() + 1, 0.0);

    double total_trip_cost = 0.0;

    // Token-based parse: whitespace/newlines are interchangeable
    int helicopter_id, num_trips;

    // Read until EOF
    while (outfile >> helicopter_id) {
        // Allow sentinel lines like "-1" to be ignored (matches the original behavior)
        if (helicopter_id == -1) {
            // Try to read next helicopter_id; continue loop
            continue;
        }

        if (!(outfile >> num_trips)) {
            cerr << "Error: Expected number of trips after helicopter ID " << helicopter_id << endl;
            return -1.0;
        }

        if (helicopter_id <= 0 || helicopter_id > static_cast<int>(data.helicopters.size())) {
            cerr << "Error: Invalid helicopter ID " << helicopter_id << endl;
            return -1.0;
        }

        const auto& helicopter = data.helicopters[helicopter_id - 1];
        Point home_city_coords = data.cities[helicopter.home_city_id - 1];

        for (int i = 0; i < num_trips; ++i) {
            int d, p, o, num_villages_in_trip;
            if (!(outfile >> d >> p >> o >> num_villages_in_trip)) {
                cerr << "Error: Expected trip header 'd p o num_villages' for heli "
                     << helicopter_id << ", trip " << (i + 1) << endl;
                return -1.0;
            }

            // Weight capacity check
            double trip_weight =
                (static_cast<double>(d) * data.packages[0].weight) +
                (static_cast<double>(p) * data.packages[1].weight) +
                (static_cast<double>(o) * data.packages[2].weight);

            if (trip_weight > helicopter.weight_capacity + 1e-9) {
                cout << "*** WARNING: Heli " << helicopter_id << ", Trip " << (i + 1)
                     << " exceeds weight capacity (" << trip_weight
                     << " > " << helicopter.weight_capacity << ")." << endl;
                constraint_violated = true;
            }

            Point current_location = home_city_coords;
            double trip_distance = 0.0;
            int total_d_dropped = 0, total_p_dropped = 0, total_o_dropped = 0;

            for (int j = 0; j < num_villages_in_trip; ++j) {
                int village_id, vd, vp, vo;
                if (!(outfile >> village_id >> vd >> vp >> vo)) {
                    cerr << "Error: Expected village entry 'id vd vp vo' for heli "
                         << helicopter_id << ", trip " << (i + 1)
                         << ", village index " << (j + 1) << endl;
                    return -1.0;
                }

                total_d_dropped += vd;
                total_p_dropped += vp;
                total_o_dropped += vo;

                if (village_id <= 0 || village_id > static_cast<int>(data.villages.size())) {
                    cerr << "Error: Invalid village ID " << village_id << endl;
                    return -1.0;
                }
                const auto& village = data.villages[village_id - 1];

                // Value capping: food (d+p) capped at 9 per person; other capped at 1 per person
                double max_food_needed = static_cast<double>(village.population) * 9.0;
                double food_room_left = max(0.0, max_food_needed - food_delivered[village_id]);

                double food_in_this_drop = static_cast<double>(vd + vp);
                double effective_food_this_drop = min(food_in_this_drop, food_room_left);

                // Prioritize package[1] "p" value, then package[0] "d"
                double effective_vp = min(static_cast<double>(vp), effective_food_this_drop);
                double value_from_p = effective_vp * data.packages[1].value;
                double remaining_effective_food = effective_food_this_drop - effective_vp;

                double effective_vd = min(static_cast<double>(vd), remaining_effective_food);
                double value_from_d = effective_vd * data.packages[0].value;

                village_values[village_id] += value_from_p + value_from_d;

                double max_other_needed = static_cast<double>(village.population) * 1.0;
                double other_room_left = max(0.0, max_other_needed - other_delivered[village_id]);
                double effective_vo = min(static_cast<double>(vo), other_room_left);
                village_values[village_id] += effective_vo * data.packages[2].value;

                // Accumulate delivered amounts regardless of capping used in value
                food_delivered[village_id] += food_in_this_drop;
                other_delivered[village_id] += static_cast<double>(vo);

                // Distance accumulation
                trip_distance += distance(current_location, village.coords);
                current_location = village.coords;
            }

            // Picked vs dropped consistency
            if (total_d_dropped > d || total_p_dropped > p || total_o_dropped > o) {
                cout << "*** WARNING: Heli " << helicopter_id << ", Trip " << (i + 1)
                     << " drops more packages than picked up." << endl;
                constraint_violated = true;
            }

            // Return to home
            trip_distance += distance(current_location, home_city_coords);

            // Per-trip distance capacity
            if (trip_distance > helicopter.distance_capacity + 1e-9) {
                cout << "*** WARNING: Heli " << helicopter_id << ", Trip " << (i + 1)
                     << " exceeds trip distance capacity (" << trip_distance
                     << " > " << helicopter.distance_capacity << ")." << endl;
                constraint_violated = true;
            }

            helicopter_total_distances[helicopter_id] += trip_distance;

            // Costs: fixed if trip has villages, plus alpha * distance
            double trip_cost = (num_villages_in_trip > 0)
                ? (helicopter.fixed_cost + (helicopter.alpha * trip_distance))
                : 0.0;

            total_trip_cost += trip_cost;
        }

        // Per-heli DMax check
        if (helicopter_total_distances[helicopter_id] > data.d_max + 1e-9) {
            cout << "*** WARNING: Heli " << helicopter_id << " exceeds DMax ("
                 << helicopter_total_distances[helicopter_id] << " > " << data.d_max << ")." << endl;
            constraint_violated = true;
        }

        // IMPORTANT: no stray getline() here. We continue token-by-token.
    }

    // Final objective
    double total_value = accumulate(village_values.begin(), village_values.end(), 0.0);
    double final_score = total_value - total_trip_cost;

    cout << "\n--- Final Calculation ---" << endl;
    cout << "Total Value Gained: " << total_value << endl;
    cout << "Total Trip Cost   : " << total_trip_cost << endl;
    cout << "Objective Score   = " << total_value << " - " << total_trip_cost
         << " = " << final_score << endl;

    if (constraint_violated) {
        cout << "\n*** WARNING: CONSTRAINTS VIOLATED. Score is invalid. ***" << endl;
        return -1.0;
    }

    cout << "\n--- All constraints satisfied. ---" << endl;
    return final_score;
}

int main(int argc, char* argv[]) {
    if (argc != 3) {
        cerr << "Usage: " << argv[0] << " <input_filename> <output_filename>" << endl;
        return 1;
    }

    try {
        double score = verifyAndCalculateScore(argv[1], argv[2]);
        cout << "\n----------------------------------------\n"
             << "FINAL SCORE: " << score
             << "\n----------------------------------------" << endl;
    } catch (const exception& e) {
        cerr << "An error occurred: " << e.what() << endl;
        return 1;
    }
    return 0;
}
