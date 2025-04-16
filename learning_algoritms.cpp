#include "pch.h"
#include "action.h"
#include <vector>
#include "global_definitions.h"
#include "state.h"



//Transtiton probability estimation with UCB1 algoritm also contains control function, this is for gambler's ruin problem
void UCB_Tprob_Estimate(vector<state*> states, int round_number)
{
	int state_number = states.size();
	int round = 1;
	int cont_taken = 1;
	int final_taken = 1;

	uniform_int_distribution<> dist(1, state_number - 2);
	std::bernoulli_distribution distributioncont(real_upprob);//p = up
	std::bernoulli_distribution distributionfinal(real_goalprob);//p = goal
	uniform_real_distribution<> dist_up(0, 1);
	uniform_real_distribution<> dist_final(0, 1);

	double temp_upnum = dist_up(generator);
	double temp_finalnum = dist_final(generator);

	double up_estimate;
	double final_estimate;

	int initialindex;
	int index;

	int cont_counter = 0;
	int final_counter = 0;
	int up_counter = 0;
	int goal_counter = 0;
	int goal_end_counter = 0;

	double success_rate;

	int total_num_of_goals = 0;

	int cont_must = 0;
	int final_must = 0;

	while (round_number > round)
	{
		initialindex = dist(generator);
		index = initialindex;

		up_estimate = temp_upnum / cont_taken + sqrt(2 * log(cont_taken) / (cont_taken + final_taken));
		final_estimate = temp_finalnum / final_taken + sqrt(2 * log(final_taken) / (cont_taken + final_taken));


		double newprobs[4] = { up_estimate , 1 - up_estimate , final_estimate, 1 - final_estimate };
		int i;
		for (i = 1; i < state_number - 1; ++i)
		{
			(*states[i]).setTprob(newprobs);

		}
		MAXPROB(threshold_maxprob, states);
		OPTIMALPROBPOLICY(states);


		if (100 * log(round) > cont_taken)
			cont_must = 1;
		if (100 * log(round) > final_taken)
			final_must = 1;
		do
		{
			//Action cont is suppose to be taken according to optimal policy
			cout << "round" << endl;
			if (((*states[index]).getmaxprobPolicy().compare("Cont") == 0 && cont_must == 1) || (final_must == 0 && cont_must == 1 && (*states[index]).getmaxprobPolicy().compare("Cont") != 0) || (final_must == 0 && cont_must == 0 && (*states[index]).getmaxprobPolicy().compare("Cont") == 0))
			{
				cont_counter++;
				if (distributioncont(generator))
				{
					up_counter++;
					index = index + 1;
					if (index == state_number - 1)
					{
						goal_end_counter++;
					}

				}
				else
					index = index - 1;
			}
			//Action final is supoose to be taken
			else
			{
				final_counter++;
				if (distributionfinal(generator))
				{
					goal_counter++;
					goal_end_counter++;
					index = state_number - 1;
				}
				else
					index = 0;
			}
		} while (!(index == state_number - 1 || index == 0));

		temp_finalnum = temp_finalnum + goal_counter;
		final_taken = final_taken + final_counter;

		temp_upnum = temp_upnum + up_counter;
		cont_taken = cont_taken + cont_counter;

		total_num_of_goals = total_num_of_goals + goal_end_counter;

		goal_counter = 0;
		final_counter = 0;
		up_counter = 0;
		cont_counter = 0;
		goal_end_counter = 0;
		round++;
	}

	success_rate = total_num_of_goals / (double)round_number;
	
	cout << "success_rate: " << success_rate << endl;
	cout << "final: " << final_estimate << endl;
	cout << "up: " << up_estimate << endl;

}


// A function that calculates regret data for Thompson sampling.Data is written on a text file
void regretCalculationThompson(vector<state*> &states)
{
	int state_number = states.size();
	int round = 1;

	const int sample = 100;
	const int maxroundnumber = 2000;

	uniform_int_distribution<> dist(1, state_number - 2);
	std::bernoulli_distribution distributioncont(real_upprob);//p = up
	std::bernoulli_distribution distributionfinal(real_goalprob);//p = goal
	uniform_real_distribution<> dist_up(0, 1);
	uniform_real_distribution<> dist_final(0, 1);

	int *oracle_policy_set = new int[state_number - 2]; // 1 for cont -1 for final

	int cumreward = 0;
	int initialindex;
	int oracle_index;
	int learner_index;

	int goal_rewardopt_counter = 0;

	int temp_goalnum = 1;
	int temp_deadnum = 1;
	int temp_downnum = 1;
	int temp_upnum = 1;

	int goal_counter = 0;
	int dead_counter = 0;

	int cont_counter = 0;
	int final_counter = 0;

	int up_counter = 0;
	int down_counter = 0;

	int goal_end_counter = 0;
	int dead_end_counter = 0;

	int total_num_of_goals = 0;

	double final_estimate;
	double up_estimate;

	int cumregret = 1;


	ofstream myfile;
	myfile.open("values.txt");


	int  cumRegret_set[maxroundnumber] = { 0 };
	double  expectedregret[maxroundnumber] = { 0.0 };


	MAXPROB(threshold_maxprob, states);
	OPTIMALPROBPOLICY(states);

	for (int i = 1; i < state_number - 1; i++)
	{
		if ((*states[i]).getmaxprobPolicy().compare("Cont") == 0)
			oracle_policy_set[i - 1] = 1;
		else
			oracle_policy_set[i - 1] = 0;
	}

	for (int a = 0; a < sample; a++)
	{
		cout << "sample" << endl;
		while (maxroundnumber >= round)
		{

			initialindex = dist(generator);
			oracle_index = initialindex;
			learner_index = initialindex;

			beta_distribution<> distfinalprob(temp_goalnum, temp_deadnum);
			beta_distribution<> distcontprob(temp_upnum, temp_downnum);

			final_estimate = quantile(distfinalprob, dist_up(generator));
			up_estimate = quantile(distcontprob, dist_up(generator));
			setallnewtprob(states, up_estimate, final_estimate);

			MAXPROB(threshold_maxprob, states);
			OPTIMALPROBPOLICY(states);

			bool flag_learn = true;
			bool flag_orac = true;
			do //Oracle and learner
			{
				//both cont
				if ((!flag_orac || oracle_policy_set[oracle_index - 1] == 1) && (!flag_learn || (*states[learner_index]).getmaxprobPolicy().compare("Cont") == 0))
				{
					if (flag_learn)
						cont_counter++;

					if (distributioncont(generator))//up
					{
						if (flag_learn)
						{
							up_counter++;
							learner_index++;
							if (learner_index == state_number - 1)
								goal_end_counter++;
						}
						if (flag_orac)
						{
							oracle_index++;
							if (oracle_index == state_number - 1)
								goal_rewardopt_counter++;
						}
					}
					else
					{
						if (flag_orac)
							oracle_index--;

						if (flag_learn)
						{
							down_counter++;
							learner_index--;
							if (learner_index == 0)
								dead_end_counter++;
						}
					}

				}

				//Action final - cont
				else if ((!flag_orac || oracle_policy_set[oracle_index - 1] == 0) && (!flag_learn || (*states[learner_index]).getmaxprobPolicy().compare("Cont") == 0))
				{
					if (flag_orac)
					{
						if (distributionfinal(generator))
						{
							oracle_index = state_number - 1;
							goal_rewardopt_counter++;
						}
						else
							oracle_index = 0;
					}

					if (flag_learn)
					{
						cont_counter++;
						if (distributioncont(generator))
						{
							up_counter++;
							learner_index++;
							if (learner_index == state_number - 1)
								goal_end_counter++;
						}
						else
						{
							down_counter++;
							learner_index--;
							if (learner_index == 0)
								dead_end_counter++;

						}
					}
				}
				//final-final
				else if ((!flag_orac || oracle_policy_set[oracle_index - 1] == 0) && ((!flag_learn || (*states[learner_index]).getmaxprobPolicy().compare("Cont") != 0)))
				{

					final_counter++;
					if (distributionfinal(generator))
					{
						if (flag_orac)
						{
							oracle_index = state_number - 1;
							goal_rewardopt_counter++;
						}

						if (flag_learn)
						{
							learner_index = state_number - 1;
							goal_end_counter++;
							goal_counter++;
						}

					}
					else
					{
						if (flag_orac)
							oracle_index = 0;


						if (flag_learn)
						{
							learner_index = 0;
							dead_end_counter++;
							dead_counter++;
						}
					}

				}
				//cont-final
				else if ((!flag_orac || oracle_policy_set[oracle_index - 1] == 1) && ((!flag_learn || (*states[learner_index]).getmaxprobPolicy().compare("Cont") != 0)))
				{
					if (flag_orac)
					{
						if (distributioncont(generator))
						{
							oracle_index++;
							if (oracle_index == state_number - 1)
								goal_rewardopt_counter++;
						}
						else
						{

							oracle_index--;
						}

					}
					if (flag_learn)
					{
						final_counter++;
						if (distributionfinal(generator))
						{

							learner_index = state_number - 1;
							goal_end_counter++;
							goal_counter++;

						}
						else
						{
							learner_index = 0;
							dead_end_counter++;
							dead_counter++;
						}
					}
				}
				if ((learner_index == state_number - 1 || learner_index == 0))
				{
					flag_learn = false;
				}
				if ((oracle_index == state_number - 1 || oracle_index == 0))
				{
					flag_orac = false;
				}
			} while (!((oracle_index == state_number - 1 || oracle_index == 0) && (learner_index == state_number - 1 || learner_index == 0)));

			temp_goalnum = temp_goalnum + goal_counter;
			temp_deadnum = temp_deadnum + dead_counter;

			temp_upnum = temp_upnum + up_counter;
			temp_downnum = temp_downnum + down_counter;

			cumregret = cumregret + goal_rewardopt_counter - goal_end_counter;
			cumRegret_set[round - 1] = cumregret;

			goal_rewardopt_counter = 0;
			up_counter = 0;
			down_counter = 0;
			goal_end_counter = 0;
			dead_end_counter = 0;
			goal_counter = 0;
			dead_counter = 0;
			round++;

		}

		cumregret = 0;
		temp_goalnum = 1;
		temp_deadnum = 1;
		temp_upnum = 1;
		temp_downnum = 1;

		for (int i = 0; i < maxroundnumber; i++)
		{
			expectedregret[i] = expectedregret[i] + cumRegret_set[i];
		}
		round = 1;
		for (int i = 0; i < maxroundnumber; i++)
		{
			cumRegret_set[i] = 0;
		}
	}

	for (int i = 0; i < maxroundnumber; i++)
	{
		expectedregret[i] = expectedregret[i] / (double)sample;
		myfile << expectedregret[i] << " ";
	}


	myfile.close();
	file2.close();
	delete[] cont_chosen_action_set;
	delete[] oracle_policy_set;
}


// Initially, I experimented with assigning zero rewards to continue actions and dead states, and a reward of 1 for the goal state. 
// However, this setup did not yield consistent results. I then modified the approach to assign reward values proportional to the 
// transition probabilities, aiming to simulate learning of maximum probability values similar to Kolobov's algorithm (like learning costs). 
// I also lowered the learning rate. While I haven’t fully tested this version, it performs significantly better than the previous ones 
// and consistently produces the correct optimal policy in my trials. The input and output structure of this function matches that of 
// the previously uploaded MAXPROB function. From my observations, using rewards proportional to transition probabilities and a decreasing 
// learning rate leads to surprisingly good results.     
void qlearning(int episode, double discount, vector<state*> states) {

	int state_number = states.size();
	double e = 0.2;
	
	std::bernoulli_distribution distributioncont(real_upprob);//p = up
	std::bernoulli_distribution  distributionfinal(real_goalprob);//p = goal
	std::bernoulli_distribution  distribution_egreedy(e);
	
	uniform_int_distribution<> dist(1, state_number - 2);
	uniform_int_distribution<> dist2(0, 1);
	
	double learningrate = 0.1;
	int index;
	int nextindex;
	int a; // 0 for cont 1 for final 
	double temp;
	double nexttemp;
	double imreward;
	int ind;
	int i;

	for (i = 0; i < episode; i++) {

		//Choose random initial state to start
		nextindex = dist(generator);
		index = nextindex;
		do {

			//For policy choose random policy either cont or final
			//Choose policy according to a 
			if (distribution_egreedy(generator))
			{

				if ((*states[index]).getqvalue((*states[index]).get_action(0)) >= (*states[index]).getqvalue((*states[index]).get_action(1)))
				{
					a = 0;
				}
				else
					a = 1;
			}
			else
			{

				if (dist2(generator) == 0)
					a = 0;
				else
					a = 1;
			}

			temp = (*states[index]).getqvalue((*states[index]).get_action(a));
			if (a == 0)//cont selected according to e-greedy policy
			{
				if (distributioncont(generator)) // tprob up chosen
				{
					nexttemp = max((*states[index + 1]).getqvalue((*states[index]).getAction(0)), (*states[index + 1]).getqvalue((*states[index]).getAction(1)));
					imreward = (*states[index]).getReward(1, (*states[index]).get_action(a));
					temp = temp + learningrate * (imreward + discount * nexttemp - temp);
					ind = 0;

				}
				else {// down chosen
					nexttemp = max((*states[index - 1]).getqvalue((*states[index]).getAction(0)), (*states[index - 1]).getqvalue((*states[index]).getAction(1)));
					imreward = (*states[index]).getReward(0, (*states[index]).get_action(a));
					temp = temp + learningrate * (imreward + discount * nexttemp - temp);
					ind = 1;

				}
			}
			else // final selected according to e-greedy policy a = 1
			{
				if (distributionfinal(generator)) // Goal state
				{
					nexttemp = max((*states[state_number - 1]).getqvalue((*states[index]).getAction(0)), (*states[state_number - 1]).getqvalue((*states[index]).getAction(1)));
					imreward = (*states[index]).getReward(1, (*states[index]).get_action(a));
					temp = temp + learningrate * (imreward + discount * nexttemp - temp);
					ind = 2;

				}
				else {   // Dead State
					nexttemp = max((*states[0]).getqvalue((*states[index]).getAction(0)), (*states[0]).getqvalue((*states[index]).getAction(1)));
					imreward = (*states[index]).getReward(0, (*states[index]).get_action(a));
					temp = temp + learningrate * (imreward + discount * nexttemp - temp);
					ind = 3;

				}
			}

			(*states[index]).setqvalue(temp, a);

			switch (ind)
			{
			case 0:
				index = index + 1;
				break;
			case 1:
				index = index - 1;
				break;
			case 2:
				index = state_number - 1;
				break;
			case 3:
				index = 0;
				break;
			}

		} while (!(index == state_number - 1 || index == 0));
	}
	for (i = 1; i < state_number - 1; ++i)
	{

		cout << "final cont value " << i << " " << (*states[i]).getqvalue(0) << endl;
		cout << "final final  " << i << "  " << (*states[i]).getqvalue(1) << endl;
		cout << endl;

		if ((*states[i]).getqvalue((*states[i]).get_action(0)) > (*states[i]).getqvalue((*states[i]).get_action(1)))
			cout << " policy Cont" << endl;
		else if ((*states[i]).getqvalue((*states[i]).get_action(0)) < (*states[i]).getqvalue((*states[i]).get_action(1)))
			cout << " policy Final" << endl;

		cout << endl;


	}
}

void TprobEstimate(vector<state*> states, int round_number)
{
	int state_number = states.size();
	int round = 1;
	int cont_taken = 1;
	int final_taken = 1;

	double temp_upnum = 0.1;
	double temp_finalnum = 0.1;

	double up_estimate;
	double final_estimate;

	int initialindex;
	int index;

	int cont_counter = 0;
	int final_counter = 0;
	int up_counter = 0;
	int goal_counter = 0;
	int goal_end_counter = 0;

	//To compare success of estimation 
	double success_rate;

	int total_num_of_goals = 0;

	int cont_must = 0;
	int final_must = 0;

	uniform_int_distribution<> dist(1, state_number - 2);
	std::bernoulli_distribution distributioncont(real_upprob);//p = up
	std::bernoulli_distribution distributionfinal(real_goalprob);//p = goal
	while (round_number > round)
	{
		//randomly choose initial state
		initialindex = dist(generator);
		index = initialindex;

		up_estimate = temp_upnum / cont_taken;
		final_estimate = temp_finalnum / final_taken;

		//Setting estimated probalities
		double newprobs[4] = { up_estimate , 1 - up_estimate , final_estimate, 1 - final_estimate };
		int i;
		for (i = 1; i < state_number - 1; ++i)
		{
			(*states[i]).setTprob(newprobs);

		}

		//Get temporary optimal policy from estimated probs	
		MAXPROB(threshold_maxprob, states);
		OPTIMALPROBPOLICY(states);

		//control function
		if (100 * log(round) > cont_taken)
			cont_must = 1;
		if (100 * log(round) > final_taken)
			final_must = 1;

		do
		{
			//Action cont is suppose to be taken according to optimal policy
			if (((*states[index]).getmaxprobPolicy().compare("Cont") == 0 && cont_must == 1) || (final_must == 0 && cont_must == 1 && (*states[index]).getmaxprobPolicy().compare("Cont") != 0) || (final_must == 0 && cont_must == 0 && (*states[index]).getmaxprobPolicy().compare("Cont") == 0))
			{
				cont_counter++;
				//Agent get to upper state as a result of cont
				if (distributioncont(generator))
				{
					up_counter++;
					index = index + 1;
					if (index == state_number - 1)
					{
						goal_end_counter++;
					}

				}
				else
					index = index - 1;
			}
			//Action final is supoose to be taken
			else
			{
				final_counter++;
				//Agent hits the goal
				if (distributionfinal(generator))
				{
					goal_counter++;
					goal_end_counter++;
					index = state_number - 1;
				}
				else
					index = 0;
			}
		} while (!(index == state_number - 1 || index == 0));

		temp_finalnum = temp_finalnum + goal_counter;
		final_taken = final_taken + final_counter;

		temp_upnum = temp_upnum + up_counter;
		cont_taken = cont_taken + cont_counter;

		total_num_of_goals = total_num_of_goals + goal_end_counter;

		goal_counter = 0;
		final_counter = 0;
		up_counter = 0;
		cont_counter = 0;
		goal_end_counter = 0;
		cont_must = 0;
		final_must = 0;
		round++;
	}

	//Success rate calculated by dividing rounds that agent hit goal state to total round number.
	success_rate = total_num_of_goals / (double)round_number;
	cout << "success_rate: " << success_rate << endl;
}



