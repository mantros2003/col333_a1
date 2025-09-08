#ifndef COST_FUNC_H
#define COST_FUNC_H

#include<vector>

#include "structures.h"

using namespace std;

double proj(const Point& p, double cx, double sy);
double nearest_into_set(int a, const vector<int>& v_idx, const ProblemData& P);
double closed_tour_span2(const vector<int>& v_idx, int s, const ProblemData& P);
double distance_travelled(int s, const vector<int>& v_idx, const ProblemData& P);
double g(int village_index, int city_index, int helicopter_index, const ProblemData& problem_data,const State& current_state, double);
double g(const ProblemData &p, const State &curr_state, int heli_id, Point start, Point end, double value, bool at_home);
double h(int helicopter_index, int curr_village_idx, const ProblemData& problem_data,const State& current_state);


#endif  // COST_FUNC_H