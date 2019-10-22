#include <time.h>
#include <stdlib.h>
#include <limits.h>
#include <math.h>


#include "ai.h"
#include "utils.h"
#include "priority_queue.h"


struct heap h;

float get_reward( node_t* n );

bool validAction(node_t* current, move_t move);
void propagateBackToFirstAction(node_t* child, propagation_t propagation);
bool lostLife(node_t* child);

/**
 * Function called by pacman.c
*/
void initialize_ai(){
	heap_init(&h);
}

/**
 * function to copy a src into a dst state
*/
void copy_state(state_t* dst, state_t* src){
	//Location of Ghosts and Pacman
	memcpy( dst->Loc, src->Loc, 5*2*sizeof(int) );

    //Direction of Ghosts and Pacman
	memcpy( dst->Dir, src->Dir, 5*2*sizeof(int) );

    //Default location in case Pacman/Ghosts die
	memcpy( dst->StartingPoints, src->StartingPoints, 5*2*sizeof(int) );

    //Check for invincibility
    dst->Invincible = src->Invincible;

    //Number of pellets left in level
    dst->Food = src->Food;

    //Main level array
	memcpy( dst->Level, src->Level, 29*28*sizeof(int) );

    //What level number are we on?
    dst->LevelNumber = src->LevelNumber;

    //Keep track of how many points to give for eating ghosts
    dst->GhostsInARow = src->GhostsInARow;

    //How long left for invincibility
    dst->tleft = src->tleft;

    //Initial points
    dst->Points = src->Points;

    //Remiaining Lives
    dst->Lives = src->Lives;

}

node_t* create_init_node( state_t* init_state ){
	node_t * new_n = (node_t *) malloc(sizeof(node_t));
	new_n->parent = NULL;
	new_n->priority = 0;
	new_n->depth = 0;
	new_n->num_childs = 0;
	copy_state(&(new_n->state), init_state);
	new_n->acc_reward =  get_reward( new_n );
	return new_n;
}


float heuristic( node_t* n ){
	float h = 0;
	float i = 0;
	float l = 0;
	float g = 0;
	//FILL IN MISSING CODE
	node_t* parent = n->parent;
	if (!parent) {
		return h;
	}
	// only assigns i = 10 if pacman was not invincible in the previous state
	// and consumed a fruit during a move to the current state
	if (n->state.Invincible > parent->state.Invincible) {
		i = 10;
	}
	// only assigns l = 10 if pacman has less lives than previous state
	// ie. lost a life making the last move
	if (n->state.Lives < parent->state.Lives) {
		l = 10;
	}
	// only assigns g = 100 if pacman lost a life and now game is over
	if (n->state.Lives == -1) {
		g = 100;
	}

	h = i - l - g;
	return h;
}

float get_reward ( node_t* n ){
	float reward = 0;
	// assigning parent pointer to compare points between states after move
	node_t* parent = n->parent;
	// if game is in first state, parent will be null, hence reward = 0
	if (!parent) {
		return reward;
	}
	reward = heuristic(n) + n->state.Points - parent->state.Points;

	float discount = pow(0.99,n->depth);

	return discount * reward;
}

/**
 * Apply an action to node n and return a new node resulting from executing the action
*/
bool applyAction(node_t* n, node_t** new_node, move_t action, propagation_t propagation){
	bool changed_dir = false;
    //FILL IN MISSING CODE
	(*new_node)->parent = n;
	// change state of new node based on action passed
    changed_dir = execute_move_t( &((*new_node)->state), action);
	// if (changed_dir) {
	// 	n->num_childs++;
	// }
	(*new_node)->depth = n->depth + 1;
	(*new_node)->priority = -1*((*new_node)->depth);
	// calculates acc_reward by calling get_reward which compares reward
	// between parent and current state
	(*new_node)->move = action;
	(*new_node)->acc_reward = n->acc_reward + get_reward(*new_node);
	// if (propagation == avg) {
	// 	if (children == 1) {
	// 		n->acc_reward = (*new_node)->acc_reward;
	// 	}
	// 	else {
	// 		n->acc_reward += (*new_node)->acc_reward;
	// 	}
	// }
	return changed_dir;
}

/**
 * Find best action by building all possible paths up to budget
 * and back propagate using either max or avg
 */

move_t get_next_move( state_t init_state, int budget, propagation_t propagation, char* stats ){
	move_t best_action;
	float best_action_score[4];
	for(unsigned i = 0; i < 4; i++)
	    best_action_score[i] = INT_MIN;

	unsigned generated_nodes = 0;
	unsigned expanded_nodes = 0;
	unsigned max_depth = 0;

	//Add the initial node
	// node <- start
	node_t* n = create_init_node( &init_state );

	//Use the max heap API provided in priority_queue.h
	// frontier <- priority queue containing 'node' only
	heap_push(&h,n);
	//FILL IN THE GRAPH ALGORITHM
	// array of explored nodes - capped by budget
	node_t** explored = (node_t**) malloc(budget*sizeof(node_t*));
	// while frontier/heap is not emptyPQ
	while (h.count) {
		// pop the highest priority node off the queue
		node_t* parent = heap_delete(&h);
		// add it to the explored array and increment counter
		explored[expanded_nodes++] = parent;
		// if there is still budget then calculate new moves/nodes
		if (expanded_nodes < budget) {
			for (int move = 0; move < MAX_MOVES; move++) {
				node_t* child = create_init_node(&parent->state);
				// if the action is valid, apply it and calculate the new Score
				// and propagate back to the first move performed (left/right/up/down)
				if (applyAction(parent, &child, move, propagation)) {
					parent->num_childs++;
					propagateBackToFirstAction(child, propagation);
					// if move leads to immediate death, do not consider
					// otherwise add it to the queue
					if (lostLife(child)) {
						free(child);
					}
					else {
						heap_push(&h, child);
						// update amount of nodes generated and max depth
						generated_nodes++;
						if (child->depth > max_depth) {
							max_depth = child->depth;
						}
					}
				}
				else {
					best_action_score[move] = INT_MIN;
					free(child);
				}
			}

		}
		else {
			break;
		}
	}
	// clearing heap once budget is used up as we cant explore those nodes
	emptyPQ(&h);
	// once the heap is empty and budget is met, iterate through explored array
	// and find the nodes with depth == 1, find the move that was taken to get
	// to that node, and update best_action_score
	float top_score = INT_MIN;
	if (propagation == max) {
		for (int i = 0; i < expanded_nodes; i++) {
			if (explored[i]->depth == 1) {
				best_action_score[explored[i]->move] = explored[i]->acc_reward;
				if (explored[i]->acc_reward > top_score) {
					top_score = explored[i]->acc_reward;
					best_action = explored[i]->move;
				}
			}
			free(explored[i]);
		}
		free(explored);
	}
	else {
		node_t* current;
		float added_reward;
		int counter[4] = {0};
		for (int i = 0; i < expanded_nodes; i++) {
			current = explored[i];
			if (current->num_childs == 0) {
				added_reward = current->acc_reward;
				while (current->parent->parent) {
					current = current->parent;
				}
				if (counter[current->move] == 0) {
					best_action_score[current->move] = added_reward;
					counter[current->move]++;
				}
				else {
					best_action_score[current->move] += added_reward;
					counter[current->move]++;
				}
			}
		}
		for (int i = 0; i < MAX_MOVES; i++) {
			if (counter[i] != 0) {
				best_action_score[i] = best_action_score[i]/counter[i];
			}
			if (best_action_score[i] > top_score) {
				top_score = best_action_score[i];
				best_action = i;
			}
		}
		for (int i = 0; i < expanded_nodes; i++) {
			free(explored[i]);
		}
		free(explored);
	}
	//tiebreaker function
	while(true) {
		int random = rand() % 4;
		if (best_action_score[random] == top_score) {
			best_action = random;
			break;
		}
	}
	sprintf(stats, "Max Depth: %d Expanded nodes: %d  Generated nodes: %d\n",max_depth,expanded_nodes,generated_nodes);
	if(best_action == left)
		sprintf(stats, "%sSelected action: Left\n",stats);
	if(best_action == right)
		sprintf(stats, "%sSelected action: Right\n",stats);
	if(best_action == up)
		sprintf(stats, "%sSelected action: Up\n",stats);
	if(best_action == down)
		sprintf(stats, "%sSelected action: Down\n",stats);

	sprintf(stats, "%sScore Left %f Right %f Up %f Down %f",stats,best_action_score[left],best_action_score[right],best_action_score[up],best_action_score[down]);
	return best_action;
}

// checks for valid actions for Dijkstra node generation
bool validAction(node_t* current, move_t move) {
	state_t* temp = (state_t*) malloc(sizeof(state_t));
	copy_state(temp, &current->state);
	bool valid = execute_move_t(temp, move);
	free(temp);
	return valid;
}
// checks if life has been lost between states
bool lostLife(node_t* child) {
	node_t* parent = child->parent;
	return child->state.Lives < parent->state.Lives;
}
//
void propagateBackToFirstAction(node_t* child, propagation_t propagation) {
	node_t* current = child;
	float acc_reward = child->acc_reward;
	// will only propagate via this function if == max
	if (propagation == max) {
			while (current->parent->parent) {
				if (current->parent->acc_reward > acc_reward) {
					acc_reward = current->parent->acc_reward;
				}
				current = current->parent;
			}
			current->acc_reward = acc_reward;
	}
}
