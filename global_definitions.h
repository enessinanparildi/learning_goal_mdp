#pragma once
#define NOMINMAX
#ifndef max
#define max(a,b) ((a) > (b) ? (a) : (b))
#endif

#include <iostream>    // using IO functions
#include <string>      // using string
#include <vector>
#include <cmath>
#include <limits>
#include <stdlib.h> 
#include <random>
#include "time.h"
#include <chrono>
#include <boost/math/distributions/beta.hpp>
#include <stack> 
#include <algorithm>
#include <fstream>
#include "action.h"
#include "state.h"

//#include <vld.h>

using namespace std;
using namespace std::chrono;
using namespace boost::math;


using namespace std;
using namespace std::chrono;

const int max_state_number = 4;
const int max_action_size = 2;
const int transition_size = 2;


const double real_goalprob = 0.51;
const double real_upprob = 0.65;


const double penalty = 0;
const double costup = 0;
const double costdown = 0;
const double goalreward = 1;
const double threshold_maxprob = 0.0001;

const double probs[4] = { real_upprob, 1 - real_upprob, real_goalprob , 1 - real_goalprob };
const int episode = 1000;

mt19937 generator;
random_device randomSeed;

class state;
class action;


void find_and_revise(vector<state*> &states, double threshold, const double* heuristic);
void Search(bool* discovered, vector<state*> &states, const int & current_state_num);
void MAXPROB(double threshold, vector<state*> &states);
void OPTIMALPROBPOLICY(vector<state*>& states);
bool outgoedgesingle(vector<state*> states, int* scc_comp, int* size);
bool outgoedgegeneral(vector<state*> states, int* scc_comp, int* size);
void eliminatetrap(vector<state*> &states, int* size, int**& scc_set);
void set_for_non_permanant_states(vector<state*> states, int* scc_comp);
int**  Tarjan(vector<state*> states, int* size);
void tarjanconnect(int* index, int* lowlink, bool* in_stack, stack<state>* S, int in, vector<state*> states, int** &scc, int* size);




