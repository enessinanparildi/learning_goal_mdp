#include "pch.h"
#include "state.h"
#include "action.h"
#include "global_definitions.h"

vector<state*> state::MDP;
int state::state_cardinality = 0;

state::state()
{
	action_size = 0;
	this->setgoalprob(0);
	state_cardinality = state_cardinality + 1;
	state_num = state_cardinality;
	MDP.push_back(this);
}

state::~state()
{
}

// Add action to state
void state::add_action(action* a)
{
	actionset.push_back(a);
	action_size = actionset.size();
}
action* state::get_action(int num) // Get action by index
{
	if (num < action_size)
		return actionset[num];
	else
	{
		action *a = new action();
		(a)->addnextstatetprob(this, 1);
		this->add_action(a);
		delete a;
		return actionset[num];
	}
}
int state::getActionIndex(action* a)
{
	for (int i = 0; i < action_size; i++)
	{
		if (actionset[i] == a)
			return i;
	}
	return -1;
}
void state::setgoalprob(double prob) {//set a new value for goal prob
	goalprob = prob;
}
double state::getgoalprob() { // get that value
	return goalprob;
}
int state::getmaxprobPolicy()
{
	return maxactionindex;
}
void state::setmaxprobPolicy(int a)
{
	maxactionindex = a;
}


