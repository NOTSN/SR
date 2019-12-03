#include "main.h"

void TIM_SetPwmPulseLift(uint32_t compare);
void TIM_SetPwmPulseRight(uint32_t compare);

void LiftMotorForward(void);
void LiftMotorBackward(void);
void RightMotorForward(void);
void RightMotorBackward(void);


void BodyForward(void);
void BodyBackward(void);
void BodyTurnLeft(void);
void BodyTurnRight(void);

int Incremental_PI_LM (int Encoder,int Target);
int Incremental_PI_RM (int Encoder,int Target);

int myabs(int a);
void LimitRange_Pwm(void);
