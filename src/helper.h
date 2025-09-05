#ifndef HELPER_H
#define HELPER_H

#include <vector>
#include <queue>
#include <set>
#include "structures.h"
#include "lp_solver.h"

/** Stores the information for a single helicopter trip */
struct Trip {
    vector<int> villages;
    bool is_complete;
    double distance_covered;
    int supplies_distributed[3];
};

// IDEA: Don't precompute the supplies being taken on a trip
// Do we even need *_left variables
/** Stores the state of a single helicopter */
struct HeliState {
    Helicopter heli;
    int dry_food_left;
    int wet_food_left;
    int other_food_left;
    // Vector of vector of ints
    // For each trip, a different vector is created
    // Each vector has the village id in order of visits
    vector<Trip> trips;
};

/** Stores the state of a single village */
struct VillageState {
    Village vill;
    int dry_food_rec;
    int wet_food_rec;
    int other_food_rec;
};

/** Combines helicopter and village states */
struct State {
    vector<HeliState> heliStates;
    vector<VillageState> villageStates;
};

/** Node for the search tree */
struct Node {
    State state;
    double g;
    double h;

    bool operator>(const Node& other) const {
        return (g + h) > (other.g + other.h);
    }
};

inline double calculate_total_heli_distance(const HeliState& heli_state, const ProblemData &problem);
double value(State &s, const ProblemData &problem);
bool is_village_satisfied(const State &s, int v);
std::vector<std::vector<int>> preprocess(const ProblemData &problem);

State expand(State &s, const ProblemData &problem);
std::vector<Node> expand_single_heli(const Node &parent_node, const ProblemData &problem, double (*g)(State), double (*h)(State));
std::vector<Node> expand(const Node &parent_node, const ProblemData &problem, double (*g)(State), double (*h)(State));

inline double calculate_total_heli_distance(const HeliState& heli_state, const ProblemData &problem) {
    double total_distance = 0.0;
    for (const Trip &trip : heli_state.trips) {
        total_distance += trip.distance_covered;
    }
    return total_distance;
}

#endif // HELPER_H