#include "pch.h"
#include "action.h"
#include "global_definitions.h"
#include "state.h"


//A function that takes a single scc component and state space as a input and returns boolean value indicating whether that component have
//outgoing edge or not with respect to current policy which is produced after value iteration.
//If it does not have any outgoing edge and do not contain goal state it become trap scc.  

bool outgoedgesingle(vector<state*> states, int* scc_comp, int* size)
{
	int state_number = states.size();
	int scc_compsize = 0;
	for (int a = 0; a < state_number; a++)
	{
		if (*(scc_comp + a) == -1)
			break;
		else
			scc_compsize++;
	}
	*size = scc_compsize;

	for (int a = 0; a < scc_compsize; a++)
	{
		int  size = (*states[*(scc_comp + a)]).get_action((*states[*(scc_comp + a)]).getmaxprobPolicy())->size;

		for (int i = 0; i < size; i++)
		{
			int count = 0;
			for (int c = 0; c < scc_compsize; c++)
				if ((*states[*(scc_comp + a)]).get_action((*states[*(scc_comp + a)]).getmaxprobPolicy())->getnextstatbyindex(i)->state_num != (*(scc_comp + c)))
					count++;
			if (count == scc_compsize)
				return true;
		}

	}
	return false;
}

//The general checking of outgoing edges for all actions
bool outgoedgegeneral(vector<state*> states, int* scc_comp, int* size)
{
	int state_number = states.size();
	int scc_compsize = 0;
	for (int a = 0; a < state_number; a++)
	{
		if (*(scc_comp + a) == -1)
			break;
		else
			scc_compsize++;
	}
	*size = scc_compsize;

	for (int a = 0; a < scc_compsize; a++)
	{
		int action_size = (*states[*(scc_comp + a)]).action_size;

		for (int i = 0; i < action_size; i++)
		{
			int  next_statesize = (*states[*(scc_comp + a)]).get_action(i)->size;

			for (int c = 0; c < next_statesize; c++)
			{
				int count = 0;
				for (int v = 0; v < scc_compsize; v++)
					if ((*states[*(scc_comp + a)]).get_action(i)->getnextstatbyindex(c)->state_num != (*(scc_comp + v)))
						count++;
				if (count == scc_compsize)
					return true;
			}
		}

	}
	return false;
}

//A function that takes scc set as a input and finds whether they are trap according to graph produced by current goal probability function.After identifying 
// we need to get permanent traps which for all states contain no action that leads to other state outside of trap whatsoever.These permanent states are
//absolute dead ends and their goal probability function is set to zero.
void eliminatetrap(vector<state*> &states, int* size, int**& scc_set)
{
	int state_number = states.size();
	scc_set = Tarjan(states, size);
	bool check;
	int scc_set_size = (*size);
	int sin_scc_size;

	for (int i = 0; i < scc_set_size; i++)
	{
		for (int a = 0; a < state_number; a++)
			cout << *(*(scc_set + i) + a) << " ";
		cout << endl;
	}

	for (int i = 0; i < scc_set_size; i++)
	{

		check = outgoedgesingle(states, *(scc_set + i), &sin_scc_size);
		cout << sin_scc_size << endl;
		cout << check << endl;
		for (int a = 0; a < sin_scc_size; a++)
		{
			int * index = (*(scc_set + i) + a);
			if (*(index) == -1)
				break;
			else
			{
				if (*(index) == state_number || check)//This is not a trap
					memset(*(scc_set + i), -1, sizeof(int)*state_number);//eliminate them 
			}
		}


	}
	bool per_trap;
	for (int i = 0; i < scc_set_size; i++)
	{
		if (**(scc_set + i) != -1)// Eliminate non-trap scc component that we indicate above
		{
			int trap_scc_size = 0;
			int* trap_scc = *(scc_set + i);
			per_trap = !(outgoedgegeneral(states, trap_scc, &trap_scc_size));
			if (per_trap)//Permanent trap the above outgoedgegeneral verify that for all states of scc component none of the action can lead outside states
			{
				for (int a = 0; a < trap_scc_size; a++)
				{
					(*states[*(trap_scc + a)]).setgoalprob(0);
				}
			}
			else // Non permanent trap there are at least one action in at least one state in the component that exits the component. 
			{
				set_for_non_permanant_states(states, trap_scc);
			}
		}
	}



}

//If particular trap is not permanent call this function.Not permanent means
//They have at least one action out of all states that leads to exit but they are not visible
//under current greddy policy.So this function sets next prob value for all states in such component
//to the highest q-value of any action in any component's states that has a chance to 
//transitioning to exit 
void set_for_non_permanant_states(vector<state*> states, int* scc_comp)
{
	int state_number = states.size();
	vector<action> exit_actions;
	int scc_compsize = 0;
	for (int a = 0; a < state_number; a++)
	{
		if (*(scc_comp + a) == state_number + 1)
			break;
		else
			scc_compsize++;
	}
	for (int a = 0; a < scc_compsize; a++)
	{
		int action_size = (*states[*(scc_comp + a)]).action_size;

		for (int i = 0; i < action_size; i++)
		{
			int  next_statesize = (*states[*(scc_comp + a)]).get_action(i)->size;

			for (int c = 0; c < next_statesize; c++)
			{
				int count = 0;
				for (int v = 0; v < scc_compsize; v++)
					if ((*states[*(scc_comp + a)]).get_action(i)->getnextstatbyindex(c)->state_num != (*(scc_comp + v)))
						count++;
				if (count == scc_compsize)
					exit_actions.push_back(*(*states[*(scc_comp + a)]).get_action(i)); // exit action one of the next state is not belong to component
			}
		}

	}
	double max = 0;
	double temp = 0;
	for (int i = 0; i < exit_actions.size(); i++)
	{
		int  next_state_exit_size = exit_actions[i].size;
		for (int c = 0; c < next_state_exit_size; c++)
			temp = temp + ((exit_actions[i]).getnextstatbyindex(c)->getgoalprob())* (exit_actions[i]).getTprobbyindex(c);
		if (temp >= max)
		{
			max = temp;
		}
		temp = 0;

	}
	for (int a = 0; a < scc_compsize; a++)
	{
		(*states[*(scc_comp + a)]).setgoalprob(max);
	}

}

//Implementation of Tarjan's algoritm to find strongly connected component for greedy graph in our MDP
//The first value iteration produce greedy graph where we choose one action with respect to greedy policy according to converged goal probability function
//from MAXPROB function, then this algorithm perform first step of identifying traps, finding scc's after finding these eliminate traps are fairly easy.
//I have used example graph depicted in the image I uploaded.Output is two dimensional array where rows are different components , columns represents which
//states are in that component.For the example graph outputs of my code is subsets that are showed in the picture.  

int**  Tarjan(vector<state*> states, int* size)
{
	int state_number = states.size();
	int** scc = new int*[state_number];
	memset(scc, NULL, sizeof(int*)*state_number);
	int* index = new int[state_number];
	memset(index, -1, sizeof(int)*state_number);
	int* lowlink = new int[state_number];
	memset(lowlink, -1, sizeof(int)*state_number);
	bool* in_stack = new bool[state_number];
	memset(in_stack, false, sizeof(bool)*state_number);
	stack<state> *S = new stack<state>();
	for (int i = 0; i < state_number; i++)
		if (index[i] == -1)
			tarjanconnect(index, lowlink, in_stack, S, i, states, scc, size);


	delete[] index;
	delete[] lowlink;
	delete[] in_stack;
	return scc;

}
void tarjanconnect(int* index, int* lowlink, bool* in_stack, stack<state>* S, int in, vector<state*> states, int** &scc, int* size)
{
	int state_number = states.size();
	static int disc = 0;
	static int num_comp = 0;
	int count = 0;
	disc++;
	index[in] = disc;
	lowlink[in] = disc;
	in_stack[in] = true;
	(S)->push((*states[in]));
	int a;
	for (int i = 0; i < (*states[in]).get_action((*states[in]).getmaxprobPolicy())->size; i++)
	{
		state* s = (states[in]->get_action((*states[in]).getmaxprobPolicy())->getnextstatbyindex(i));
		a = s->state_num;
		if (index[a] == -1)
		{
			tarjanconnect(index, lowlink, in_stack, S, a, states, scc, size);
			lowlink[in] = std::min(lowlink[a], lowlink[in]);

		}
		else if (in_stack[a] == true)
		{
			lowlink[in] = std::min(lowlink[in], lowlink[a]);

		}

	}
	if (index[in] == lowlink[in])
	{

		*(scc + num_comp) = new int[state_number];
		int * curr = *(scc + num_comp);
		memset(curr, -1, sizeof(int) * state_number);
		num_comp++;

		int x;
		do
		{
			in_stack[((*S).top()).state_num] = false;
			*(curr + count) = ((*S).top().state_num);
			x = ((*S).top().state_num);
			count++;
			cout << (*S).top().state_num << " ";
			(S)->pop();
		} while (S->empty() == false && x != in);
		count = 0;
		(*size) = num_comp;
		cout << endl;
	}


}
