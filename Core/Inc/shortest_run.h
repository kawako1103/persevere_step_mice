/*
 * shortest_run.h
 *
 *  Created on: 2023/02/07
 *      Author: kawaguchitakahito
 */

#ifndef INC_SHORTEST_RUN_H_
#define INC_SHORTEST_RUN_H_

void short_step_number_revised(void);
void short_least_step_judgement_and_action_decision(void);
void short_action_based_on_direction_decision_and_coordinate_update(void);
void imaginary_run_and_pass_determination(void);
void shortest_run_action_based_on_direction_decision_and_coordinate_update(float);
void shortest_run(float);
void after_explore_shortes_run(float);

extern int pass[255];
extern int nmax;
#endif /* INC_SHORTEST_RUN_H_ */
