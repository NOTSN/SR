#include "motor.h"

/**********************************************************************************
  * @brief  左电机PWM脉宽赋值函数 即TIM2通道一
  * @param  pulse 脉宽值
  * @retval None
***********************************************************************************/
void TIM_SetPwmPulseLift(uint32_t pulse)
{
    TIM2->CCR1=pulse; 
}
/**********************************************************************************
  * @brief  右电机PWM脉宽赋值函数 即TIM2通道二
  * @param  pulse 脉宽值
  * @retval None
***********************************************************************************/
void TIM_SetPwmPulseRight(uint32_t pulse)
{
    TIM2->CCR2=pulse; 
}
/**********************************************************************************
  * @brief  左电机正转
  * @param  None
  * @retval None
***********************************************************************************/
void LiftMotorForward(void)
{
	HAL_GPIO_WritePin(LMotorA_GPIO_Port,LMotorA_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LMotorB_GPIO_Port,LMotorB_Pin,GPIO_PIN_SET);
}
/**********************************************************************************
  * @brief  左电机反转
  * @param  None
  * @retval None
***********************************************************************************/
void LiftMotorBackward(void)
{
	HAL_GPIO_WritePin(LMotorA_GPIO_Port,LMotorA_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LMotorB_GPIO_Port,LMotorB_Pin,GPIO_PIN_RESET);
}
/**********************************************************************************
  * @brief  右电机正转
  * @param  None
  * @retval None
***********************************************************************************/
void RightMotorForward(void)
{
	HAL_GPIO_WritePin(RMotorA_GPIO_Port,RMotorA_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RMotorB_GPIO_Port,RMotorB_Pin,GPIO_PIN_SET);
}
/**********************************************************************************
  * @brief  右电机反转
  * @param  None
  * @retval None
***********************************************************************************/
void RightMotorBackward(void)
{
	HAL_GPIO_WritePin(RMotorA_GPIO_Port,RMotorA_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(RMotorB_GPIO_Port,RMotorB_Pin,GPIO_PIN_RESET);
}
/**********************************************************************************
  * @brief  整车向前
  * @param  None
  * @retval None
***********************************************************************************/
void BodyForward(void)
{
	LiftMotorForward();
	RightMotorForward();
}
/**********************************************************************************
  * @brief  整车向后
  * @param  None
  * @retval None
***********************************************************************************/
void BodyBackward(void)
{
	LiftMotorBackward();
	RightMotorBackward();
}
/**********************************************************************************
  * @brief  整车左转
  * @param  None
  * @retval None
***********************************************************************************/
void BodyTurnLeft(void)
{
	LiftMotorBackward();
	RightMotorForward();
}
/**********************************************************************************
  * @brief  整车右转
  * @param  None
  * @retval None
***********************************************************************************/
void BodyTurnRight(void)
{
	LiftMotorForward();
	RightMotorBackward();
}
/**********************************************************************************
函数功能：增量PI控制器  左电机
入口参数：编码器测量值，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_LM (int Encoder,int Target)
{ 	
   float Kp=20,Ki=30;	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                //计算偏差
	 Pwm+=Kp*(Bias-Last_bias)+Ki*Bias;   //增量式PI控制器
	 Last_bias=Bias;	                   //保存上一次偏差 
	 return Pwm;                         //增量输出
}
/**********************************************************************************
函数功能：增量PI控制器  右电机
入口参数：编码器测量值，目标速度
返回  值：电机PWM
**************************************************************************/
int Incremental_PI_RM (int Encoder,int Target)
{ 	
   float Kp=20,Ki=30;	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                //计算偏差
	 Pwm+=Kp*(Bias-Last_bias)+Ki*Bias;   //增量式PI控制器
	 Last_bias=Bias;	                   //保存上一次偏差 
	 return Pwm;                         //增量输出
}
/**************************************************************************
函数功能：绝对值函数
入口参数：int
返回  值：unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
函数功能：限制PWM赋值 
入口参数：无
返回  值：无
**************************************************************************/
void LimitRange_Pwm(void)
{	
	  int Amplitude=7100;    //===PWM满幅是7200 限制在7100
	
}
