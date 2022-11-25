/*
 * Tasks.h
 *
 *  Created on: Nov 25, 2022
 *      Author: Robby
 */

#ifndef TASKS_TASKS_H_
#define TASKS_TASKS_H_

void Task_Init(void);
void Task_RunRx(void *arg);
void Task_RunDefault(void *arg);
void Task_RunEncoder(void *arg);
void Task_RunHeartbeat(void *arg);
void Task_RunMotorControl(void *arg);


#endif /* TASKS_TASKS_H_ */
