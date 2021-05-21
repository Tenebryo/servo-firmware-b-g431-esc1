/*
 * user_main.h
 *
 *  Created on: May 18, 2021
 *      Author: Sam Blazes
 */

#ifndef APPLICATION_INCLUDE_USER_MAIN_H_
#define APPLICATION_INCLUDE_USER_MAIN_H_


void MAIN_Init(void);
void MAIN_Loop(void);


extern uint32_t encoder_count_position;
extern uint32_t encoder_mec_position;

#endif /* APPLICATION_INCLUDE_USER_MAIN_H_ */
