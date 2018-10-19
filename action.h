#pragma once
#include <iostream>    // using IO functions
#include <vector>
#pragma once

#include "state.h"
#include "global_definitions.h"

using std::vector;
class state;
class action
{
public:
	action();
	~action();
	vector<state*> nextstates;
	vector<double> transitionprob;
	int size; //Size of next states

	void addnextstatetprob(state* s, double prob);
	double getTprobbynextstate(state* s);
	double getTprobbyindex(int i);
	state* getnextstatbyindex(int i);
	void modifyTprob(double* tprobset, int size);
};

