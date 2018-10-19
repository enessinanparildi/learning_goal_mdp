#pragma once
#include "action.h"
#include <vector>
#include "global_definitions.h"

class action;
using std::vector;


class state
{
public:
	state();
	~state();
	static vector<state*> MDP;
	static int state_cardinality;
	vector<action*> actionset; // Contain set of actions a particular state have
	int state_num; // The position or index of state in MDP
	int action_size; // How many action does a state have
	double goalprob; // Probability  value for VI or heuristic search
	int maxactionindex; //Index of optimal action in actionset container
	void add_action(action* a);
	action* get_action(int num);
	int getActionIndex(action* a);
	void setgoalprob(double prob);
	double getgoalprob();
	int getmaxprobPolicy();
	void setmaxprobPolicy(int a);
};


