#include "pch.h"
#include "action.h"
#include <iostream>
#include "state.h"


action::action()
{
	size = 0;
}
action::~action()
{

}
void action::addnextstatetprob(state* s, double prob)//Adding a next state and tprob to action
{
	nextstates.push_back(s);
	transitionprob.push_back(prob);
	size++;
}
double action::getTprobbynextstate(state* s)//Recieve Tprob by states
{
	for (int i = 0; i < size; i++)
	{
		if (s != NULL)
		{
			if ((nextstates[i]) == (s))
				return transitionprob[i];
		}
	}
	return 0;
}
double action::getTprobbyindex(int i)
{
	return transitionprob[i];
}
state* action::getnextstatbyindex(int i)
{
	return nextstates[i];
}
void action::modifyTprob(double *tprobset, int size)
{
	if (size == this->size)
	{
		for (int i = 0; i < size; i++)
			transitionprob[i] = *(tprobset + i);
	}
	else
		std::cout << "invalid" << std::endl;
}
