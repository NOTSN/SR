#include "motor.h"

/**********************************************************************************
  * @brief  ����PWM����ֵ���� ��TIM2ͨ��һ
  * @param  pulse ����ֵ
  * @retval None
***********************************************************************************/
void TIM_SetPwmPulseLift(uint32_t pulse)
{
    TIM2->CCR1=pulse; 
}
/**********************************************************************************
  * @brief  �ҵ��PWM����ֵ���� ��TIM2ͨ����
  * @param  pulse ����ֵ
  * @retval None
***********************************************************************************/
void TIM_SetPwmPulseRight(uint32_t pulse)
{
    TIM2->CCR2=pulse; 
}
/**********************************************************************************
  * @brief  ������ת
  * @param  None
  * @retval None
***********************************************************************************/
void LiftMotorForward(void)
{
	HAL_GPIO_WritePin(LMotorA_GPIO_Port,LMotorA_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LMotorB_GPIO_Port,LMotorB_Pin,GPIO_PIN_SET);
}
/**********************************************************************************
  * @brief  ������ת
  * @param  None
  * @retval None
***********************************************************************************/
void LiftMotorBackward(void)
{
	HAL_GPIO_WritePin(LMotorA_GPIO_Port,LMotorA_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LMotorB_GPIO_Port,LMotorB_Pin,GPIO_PIN_RESET);
}
/**********************************************************************************
  * @brief  �ҵ����ת
  * @param  None
  * @retval None
***********************************************************************************/
void RightMotorForward(void)
{
	HAL_GPIO_WritePin(RMotorA_GPIO_Port,RMotorA_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RMotorB_GPIO_Port,RMotorB_Pin,GPIO_PIN_SET);
}
/**********************************************************************************
  * @brief  �ҵ����ת
  * @param  None
  * @retval None
***********************************************************************************/
void RightMotorBackward(void)
{
	HAL_GPIO_WritePin(RMotorA_GPIO_Port,RMotorA_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(RMotorB_GPIO_Port,RMotorB_Pin,GPIO_PIN_RESET);
}
/**********************************************************************************
  * @brief  ������ǰ
  * @param  None
  * @retval None
***********************************************************************************/
void BodyForward(void)
{
	LiftMotorForward();
	RightMotorForward();
}
/**********************************************************************************
  * @brief  �������
  * @param  None
  * @retval None
***********************************************************************************/
void BodyBackward(void)
{
	LiftMotorBackward();
	RightMotorBackward();
}
/**********************************************************************************
  * @brief  ������ת
  * @param  None
  * @retval None
***********************************************************************************/
void BodyTurnLeft(void)
{
	LiftMotorBackward();
	RightMotorForward();
}
/**********************************************************************************
  * @brief  ������ת
  * @param  None
  * @retval None
***********************************************************************************/
void BodyTurnRight(void)
{
	LiftMotorForward();
	RightMotorBackward();
}
/**********************************************************************************
�������ܣ�����PI������  ����
��ڲ���������������ֵ��Ŀ���ٶ�
����  ֵ�����PWM
��������ʽ��ɢPID��ʽ 
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  �Դ����� 
pwm�����������
�����ǵ��ٶȿ��Ʊջ�ϵͳ���棬ֻʹ��PI����
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_LM (int Encoder,int Target)
{ 	
   float Kp=20,Ki=30;	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                //����ƫ��
	 Pwm+=Kp*(Bias-Last_bias)+Ki*Bias;   //����ʽPI������
	 Last_bias=Bias;	                   //������һ��ƫ�� 
	 return Pwm;                         //�������
}
/**********************************************************************************
�������ܣ�����PI������  �ҵ��
��ڲ���������������ֵ��Ŀ���ٶ�
����  ֵ�����PWM
**************************************************************************/
int Incremental_PI_RM (int Encoder,int Target)
{ 	
   float Kp=20,Ki=30;	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                //����ƫ��
	 Pwm+=Kp*(Bias-Last_bias)+Ki*Bias;   //����ʽPI������
	 Last_bias=Bias;	                   //������һ��ƫ�� 
	 return Pwm;                         //�������
}
/**************************************************************************
�������ܣ�����ֵ����
��ڲ�����int
����  ֵ��unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
�������ܣ�����PWM��ֵ 
��ڲ�������
����  ֵ����
**************************************************************************/
void LimitRange_Pwm(void)
{	
	  int Amplitude=7100;    //===PWM������7200 ������7100
	
}
