
/* Includes ------------------------------------------------------------------*/
#include "pid.h"
#include "stm32f4xx.h"

#define ABS(x)		((x>0)? x: -x) 

/*ensure that the input data in within the correct range*/
#define LimitMax(input, max) \
    if (input > max)         \
    {                        \
        input = max;         \
    } else if (input < -max) \
    {                        \
        input = -max;        \
    }

extern int isMove;
PID_TypeDef pid_pitch,pid_pithch_speed,pid_roll,pid_roll_speed,pid_yaw_speed;
PID_TypeDef drive_motor_pid[4]; // 4 motor pid
/**
 * @brief  PID struct data init
 * @param[out]  pid: PID struct data point
 * @param[in]   mode: PID_POSITION: normal pid
 *              PID_DELTA: delta pid
 * @param[in]   PID: 0: kp, 1: ki, 2: kd
 * @param[in]   maxout: pid max output
 * @param[in]   intergral_limit: pid intergral limit, max iout
 * @retval      none
 */
static void pid_param_init(     // 參數初始化
	PID_TypeDef * pid, 
	PID_ID   id,
	uint16_t maxout,
	uint16_t intergral_limit,
	float deadband,
	uint16_t period,
	int16_t  max_err,
	int16_t  target,

	float 	kp, 
	float 	ki, 
	float 	kd)
{
	pid->id = id;		
	
	pid->ControlPeriod = period;             //沒用到
	pid->DeadBand = deadband;
	pid->IntegralLimit = intergral_limit;
	pid->MaxOutput = maxout;
	pid->Max_Err = max_err;
	pid->target = target;                   // 目標值
	
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	
	pid->output = 0;
}

/*中途更改參數設定--------------------------------------------------------------*/
static void pid_reset(PID_TypeDef * pid, float kp, float ki, float kd)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
}

/*PID計算---------------------------------------------------------------------*/
static float pid_calculate(PID_TypeDef* pid, float measure)//, int16_t target)
{
    // uint32_t time,lasttime;
	
	pid->lasttime = pid->thistime;
	pid->thistime = HAL_GetTick();
	pid->dtime = pid->thistime-pid->lasttime;
    // pid->target = target;

    pid->measure = measure;                         //測量值等於本次最新測量值 : 目標速度
	pid->last_err  = pid->err;                      //上次誤差=本次最新誤差 : 更新前一次誤差
	pid->last_output = pid->output;                 //上次輸出=本次最新輸出
	
	pid->err = pid->target - pid->measure;          //計算當前誤差    誤差值=目標值-測量值

	if((ABS(pid->err) > pid->DeadBand))             //誤差是否大於(進入)死區，如果進入則直接跳過，返回上一次的output結果
	{
		pid->pout = pid->kp * pid->err;             //p輸出為Kp*誤差
		pid->iout += (pid->ki * pid->err);          //i輸出為i+Ki*誤差

		pid->dout =  pid->kd * (pid->err - pid->last_err);

        //計分是否超出限制
		if(pid->iout > pid->IntegralLimit)
			pid->iout = pid->IntegralLimit;
		if(pid->iout < - pid->IntegralLimit)
			pid->iout = - pid->IntegralLimit;

        //pid輸出和
		pid->output = pid->pout + pid->iout + pid->dout;

        //限制輸出的大小
		//pid->output = pid->output*0.7f + pid->last_output*0.3f;  //�˲���
		if(pid->output>pid->MaxOutput)         
		{
			pid->output = pid->MaxOutput;
		}
		if(pid->output < -(pid->MaxOutput))
		{
			pid->output = -(pid->MaxOutput);
		}
	
	}
	return pid->output;
}

/*initialize the function pointers in the data structure (3 functions)------------------------------------------*/
void pid_init(PID_TypeDef* pid)
{
	pid->f_param_init = pid_param_init;
	pid->f_pid_reset = pid_reset;
	pid->f_cal_pid = pid_calculate;
}

