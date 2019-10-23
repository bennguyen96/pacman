#ifndef __AI__
#define __AI__
#define MAX_MOVES 4

#include <stdint.h>
#include <unistd.h>
#include "node.h"
#include "priority_queue.h"
#include <time.h>

int game_max_depth;
int game_gen_nodes;
int game_exp_nodes;
int game_max_value;
double total_search_time;

void initialize_ai();

move_t get_next_move( state_t init_state, int budget, propagation_t propagation, char* stats );

#endif
