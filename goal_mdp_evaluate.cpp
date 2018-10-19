#include "pch.h"
#include "action.h"
#include <vector>
#include "global_definitions.h"
#include "state.h"


int main() {
	state s0;
	state s1;
	state s2;
	state s3;
	state s4;
	state goal;

	action a0;
	action a1cont;
	action a1final;
	action a2cont;
	action a2final;
	action a3cont;
	action a3final;
	action a4cont;
	action a4final;
	action agoal;

	a0.addnextstatetprob(&s0, 1);

	a1cont.addnextstatetprob(&s0, 1 - real_upprob);
	a1cont.addnextstatetprob(&s2, real_upprob);
	a1final.addnextstatetprob(&s0, 1 - real_goalprob);
	a1final.addnextstatetprob(&goal, real_goalprob);

	a2cont.addnextstatetprob(&s1, 1 - real_upprob);
	a2cont.addnextstatetprob(&s3, real_upprob);

	a2final.addnextstatetprob(&s0, 1 - real_goalprob);
	a2final.addnextstatetprob(&goal, real_goalprob);

	a3cont.addnextstatetprob(&s2, 1 - real_upprob);
	a3cont.addnextstatetprob(&s4, real_upprob);

	a3final.addnextstatetprob(&s0, 1 - real_goalprob);
	a3final.addnextstatetprob(&goal, real_goalprob);

	a4cont.addnextstatetprob(&s3, 1 - real_upprob);
	a4cont.addnextstatetprob(&goal, real_upprob);

	a4final.addnextstatetprob(&s0, 1 - real_goalprob);
	a4final.addnextstatetprob(&goal, real_goalprob);


	agoal.addnextstatetprob(&goal, 1);

	s0.add_action(&a0);

	s1.add_action(&a1cont);
	s1.add_action(&a1final);

	s2.add_action(&a2cont);
	s2.add_action(&a2final);

	s3.add_action(&a3cont);
	s3.add_action(&a3final);

	s4.add_action(&a4cont);
	s4.add_action(&a4final);


	goal.add_action(&agoal);


	s0.setmaxprobPolicy(0);
	s1.setmaxprobPolicy(0);
	s2.setmaxprobPolicy(0);
	s3.setmaxprobPolicy(0);
	s4.setmaxprobPolicy(0);

	goal.setmaxprobPolicy(0);

	generator.seed(time(NULL));
	goal.setgoalprob(1);

	double  heuristic[] = { 0, 1 ,1, 1, 1, 1 };

	find_and_revise(goal.MDP, 0.0001, heuristic);

	system("pause");

	return 0;
}

void find_and_revise(vector<state*> &states, double threshold, const double* heuristic)
{
	int state_number = states.size();
	int initial_state_num = 2;//Initial index number of state
	bool *discovered = new bool[states.size()]; // visited nodes are labeled so that it won't search visited nodes again
	double *previous_prob = new double[states.size()]; //Previous Prob helps us to check convergence
	//this for loop is for setting the values with initial heuristic
	for (int i = 0; i < state_number; i++)
		states[i]->setgoalprob(*(heuristic + i));
	int check = 1;

	//this loop continues until convergence ends. It forces to enter search function more than once
	while (check)
	{
		memset(discovered, false, sizeof(bool)*state_number);//fill the all discovered array with false 
		//this for loop for non updated goal probabilities,previous probs are saved in order to compare for convergence
		for (int i = 0; i < state_number; i++)
			*(previous_prob + i) = states[i]->getgoalprob();

		OPTIMALPROBPOLICY(states);//Makes an Mdp to a greedy graph
		Search(discovered, states, initial_state_num);//Call search for finds and updates goal probabilities of nodes
		// this for loop checks convergence for all states
		for (int i = 0; i < state_number - 1; ++i)
		{
			if (*(discovered + i))
			{
				if (fabs(*(previous_prob + i) - (*states[i]).getgoalprob()) > threshold)
				{
					check = 1;//while loop continue or not
					break;
				}
				else
					check = 0;
			}
		}
	}

	OPTIMALPROBPOLICY(states);

	delete[] discovered;
	delete[] previous_prob;


}

//finds and updates values of states
void Search(bool* discovered, vector<state*> &states, const int & current_state_num)
{
	*(discovered + current_state_num) = true;//Labeling is done in here

	//this part is for update goal probability function values
	double curr = 0;
	double max = 0;
	for (int b = 0; b < (*states[current_state_num]).action_size; b++)
	{
		if ((*states[current_state_num]).get_action(b) != NULL)
		{

			for (int c = 0; c < (*states[current_state_num]).get_action(b)->size; c++)
			{
				double a = (*states[current_state_num]).get_action(b)->getTprobbyindex(c);
				double d = (*states[current_state_num]).get_action(b)->getnextstatbyindex(c)->getgoalprob();
				curr = curr + a * d;
				//((*states[current_state_num]).getAction(b)->getTprobbyindex(c))*((*states[current_state_num]).actionset[b]->getnextstatbyindex(c))->getgoalprob();
			}


		}
		if (curr >= max)
			max = curr;
		curr = 0;
	}
	(*states[current_state_num]).setgoalprob(max);

	//***************************
	//Decides that if search should keep iterating by checking the adjustent nodes' labels of discovered property   
	for (int b = 0; b < (*states[current_state_num]).get_action((*states[current_state_num]).getmaxprobPolicy())->size; b++)
	{
		int num = (*states[current_state_num]).get_action((*states[current_state_num]).getmaxprobPolicy())->getnextstatbyindex(b)->state_num;
		if (!*(discovered + num))
			Search(discovered, states, num);
	}

}

//Implement value iteration for general mdp it starts with zero initial prob values except goal which is set to one since we assign that state as a goal.
//It assumes traps were already identified and their prob values is set to zero.Algorithm does not modify for goal and dead states. 

void MAXPROB(double threshold, vector<state*> &states)
{
	// Contain previous goal probability function set
	int state_number = states.size();
	double *oldprob = new double[state_number - 2];
	int i;
	double curr = 0;
	int check = 1;
	double max = 0;
	int max_i;
	do {
		for (i = 0; i < state_number - 1; ++i)
		{
			oldprob[i - 1] = states[i]->goalprob;
		}
		//We could have exclude traps or in other words dead states from bellman updating, but letting them will not change anything and simplifies this code
		//As long as we assume they are correctly identified and their initial is zero.At the end it is still zero.	
		for (i = 0; i < state_number - 1; ++i)
		{

			for (int b = 0; b < (*states[i]).action_size; b++)
			{

				for (int c = 0; c < (*states[i]).actionset[b]->size; c++)
				{
					curr = curr + ((*states[i]).actionset[b]->getTprobbyindex(c))*((*states[i]).actionset[b]->getnextstatbyindex(c))->getgoalprob();

				}
				if (curr >= max)
				{
					max = curr;
					max_i = b;
				}
				curr = 0;
			}
			(*states[i]).setgoalprob(max);
			max = 0;
		}
		//checking for convergence for every state 
		for (i = 1; i < state_number - 1; ++i)
		{
			if (fabs(oldprob[i - 1] - (*states[i]).getgoalprob()) > threshold)
			{
				check = 1;
				break;
			}
			else
				check = 0;
		}

	} while (check == 1);

	delete[] oldprob;

}

//Determines optimal policies from goal probability functions for each state should be used after maxprob 
void OPTIMALPROBPOLICY(vector<state*>& states)
{
	int state_number = states.size();
	double curr = 0;
	double max = 0;
	double max_i = 0;
	int i;
	for (i = 1; i < state_number - 1; ++i)
	{

		for (int b = 0; b < (*states[i]).action_size; b++)
		{
			for (int c = 0; c < (*states[i]).actionset[b]->size; c++)
				curr = curr + ((*states[i]).actionset[b]->getTprobbyindex(c))*((*states[i]).actionset[b]->getnextstatbyindex(c))->getgoalprob();
			if (curr >= max)
			{
				max = curr;
				max_i = b;
			}
			curr = 0;

		}
		(*states[i]).setmaxprobPolicy(max_i);
		cout << "state " << i << "  " << max_i << endl;
	}
}
