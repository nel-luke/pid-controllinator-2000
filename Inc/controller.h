//
// Created by Luke Nel on 17/05/2023.
//

#ifndef PID_CONTROLLER_CONTROLLER_H
#define PID_CONTROLLER_CONTROLLER_H

void Controller_Init(void);
void Controller_Welcome(void);
void Controller_ADC_Done(void);
void Controller_Change_State(void);
void Controller_Change_Control(void);
void Controller_Reset_Enc_Cooldown(void);
void Controller_Reset_Btn_Cooldown(void);

#endif //PID_CONTROLLER_CONTROLLER_H
