/*
Complete guide to use smart CAN:
Learn CAN here:

Quick guide:
1. Include "can.h" in your file.
2. Create a Can variable (e.g. Can device;)
3. Initialize Can variable by calling Device_initialize(parameters).
Note that you would've to fill in the paramters yourself according to
your needs. For details refer to complete guide.
4. Place the function Can_Receive(parameters) into void CAN1_RX0_IRQHandler(void)
, which is located in the file "stm32f4xx_it.c".
5. In main, call CAN_Initialize somewhere below MX_CAN1_Init.
6. Lastly, send messages to your Can device by calling Can_Transmit(parameters).
For details refer to complete guide.
*/

/* ========== includes ========== */
#include "can.h"
#include <stdio.h>
#include <stdlib.h>
#include "define.h"
#include "bsp_imu.h"
/* ============================== */
#define VALUE 14
#define value 11
#define VALUEY 12
#define valuey 12
int error_sample;
/* ========== can bus ========== */
CAN_HandleTypeDef hcan1;
/* ============================= */

/* pid */
volatile Encoder CM1Encoder;
volatile Encoder CM2Encoder;
volatile Encoder CM3Encoder;
volatile Encoder CM4Encoder; 
volatile Encoder GMYawEncoder=YAW_ENCODER_DEFAULT;
volatile Encoder GMPitchEncoder=PITCH_ENCODER_DEFAULT;
PID_Regulator_t CM1SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM2SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM3SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM4SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t GMY_SPEEDPID=GIMBAL_SPEED_PID_DEFAULT;
PID_Regulator_t GMY_ANGELPID=GIMBAL_SPEED_PID_DEFAULT;
PID_Regulator_t GMP_SPEEDPID=GIMBAL_SPEED_PID_DEFAULT;
PID_Regulator_t GMP_ANGELPID=GIMBAL_SPEED_PID_DEFAULT;
offset off;
float PID[20]={0.01,0.02,0.04,0.05,0.1,0.2,0.3,0.4,0.5,1,2,3.2,4,5,10,20,30,40,50,100};
float OUTPUTY,OUTPUTP;
//---//

/* ========== headers ========== */
CAN_TxHeaderTypeDef can1TxHeader0;
CAN_TxHeaderTypeDef test;
CAN_TxHeaderTypeDef can1TxHeader1;
CAN_RxHeaderTypeDef can1RxHeader;
/* ============================= */

/* ========== filters ========== */
CAN_FilterTypeDef can1Filter;
/* ============================= */

/* ========== buffers ========== */
uint8_t canTxMsg0[8] = {0};
uint8_t canTxMsg1[8] = {0};
uint32_t can_count=0;
/* ============================= */
extern imu_t imu;
extern filter fil;
/* ========== CAN initialize functions ========== */
void CAN_Initialize(void)
{
	hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	
	
	
	can1TxHeader0.IDE = CAN_ID_STD;
	can1TxHeader0.StdId = 0x200;
	can1TxHeader0.DLC = 8;
	
	test.IDE = CAN_ID_STD;
	test.StdId = 0x200;
	test.DLC = 8;
	
	can1TxHeader1.IDE = CAN_ID_STD;
	can1TxHeader1.StdId = 0x1FF;
	can1TxHeader1.RTR = CAN_RTR_DATA;
	can1TxHeader1.DLC = 8;
	
	
	can1Filter.FilterActivation = ENABLE;
	can1Filter.FilterMode = CAN_FILTERMODE_IDMASK;
	can1Filter.FilterScale = CAN_FILTERSCALE_32BIT;
	can1Filter.FilterFIFOAssignment = CAN_FilterFIFO0;
	can1Filter.FilterIdHigh = 0x0000;
	can1Filter.FilterIdLow = 0x0000;
	can1Filter.FilterBank = 0;
	HAL_CAN_ConfigFilter(&hcan1,&can1Filter);
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_FULL);
	HAL_CAN_Start(&hcan1);
}
/* ============================================== */
//
//
//
//
//
/* Stuff below this line is the old implementation of CAN */
//
//
//
//
//
/* ========== CAN user/transmit functions ========== */
void CAN_SendMsg(CAN_HandleTypeDef* hcan,CAN_TxHeaderTypeDef *canTxHeader,uint8_t* canMsg)
{
	HAL_CAN_AddTxMessage(hcan,canTxHeader,canMsg,(void*)CAN_TX_MAILBOX0);
}
// use set_CM_speed for chassis test. However, this function will soon be degraded >:)
void set_CM_speed(int16_t cm1_iq,int16_t cm2_iq,int16_t cm3_iq,int16_t cm4_iq)
{
    canTxMsg0[0] = (uint8_t)(cm1_iq >> 8);
    canTxMsg0[1] = (uint8_t)cm1_iq;
    canTxMsg0[2] = (uint8_t)(cm2_iq >> 8);
    canTxMsg0[3] = (uint8_t)cm2_iq;
    canTxMsg0[4] = (uint8_t)(cm3_iq >> 8);
    canTxMsg0[5] = (uint8_t)cm3_iq;
    canTxMsg0[6] = (uint8_t)(cm4_iq >> 8);
    canTxMsg0[7] = (uint8_t)cm4_iq;
    CAN_SendMsg(&hcan1,&can1TxHeader0,canTxMsg0);
}
/* ================================================= */
void set_GM_speed(int16_t gm1_iq,int16_t gm2_iq)
{
	  canTxMsg1[0] = (uint8_t)(gm1_iq >> 8);
    canTxMsg1[1] = (uint8_t)gm1_iq;
    canTxMsg1[2] = (uint8_t)(gm2_iq >> 8);
    canTxMsg1[3] = (uint8_t)gm2_iq;
	  CAN_SendMsg(&hcan1,&can1TxHeader1,canTxMsg1);
}
/* ========== CAN receive functions ========== */
void CanReceiveMsgProcess(CAN_RxHeaderTypeDef *rxHeader,uint8_t* msg)
{      

	can_count++;
		switch(rxHeader->StdId)
		{
				case CAN_BUS2_MOTOR1_FEEDBACK_MSG_ID:
				{
					(can_count<=50) ? GetEncoderBias(&CM1Encoder ,rxHeader,msg):EncoderProcess(&CM1Encoder ,msg);					
				}break;
				case CAN_BUS2_MOTOR2_FEEDBACK_MSG_ID:
				{
					(can_count<=50) ? GetEncoderBias(&CM2Encoder ,rxHeader,msg):EncoderProcess(&CM2Encoder ,msg);
				}break;
				case CAN_BUS2_MOTOR3_FEEDBACK_MSG_ID:
				{
					(can_count<=50) ? GetEncoderBias(&CM3Encoder ,rxHeader,msg):EncoderProcess(&CM3Encoder ,msg);
				}break;
				case CAN_BUS2_MOTOR4_FEEDBACK_MSG_ID:
				{
					(can_count<=50) ? GetEncoderBias(&CM4Encoder ,rxHeader,msg):EncoderProcess(&CM4Encoder ,msg);
				}break;
				case CAN_BUS2_MOTOR5_FEEDBACK_MSG_ID:
				{
					(can_count<=50) ? GetEncoderBias(&GMYawEncoder ,rxHeader,msg):EncoderProcess(&GMYawEncoder ,msg);
				}break;
				
				case CAN_BUS2_MOTOR6_FEEDBACK_MSG_ID:
				{	
					(can_count<=50) ? GetEncoderBias(&GMPitchEncoder ,rxHeader,msg):EncoderProcess(&GMPitchEncoder ,msg);
				}break;		
				case CAN_BUS2_MOTOR7_FEEDBACK_MSG_ID:
				{
				}break;
				case CAN_BUS2_MOTOR8_FEEDBACK_MSG_ID:
				{
				}break;
				
		}

}

void Gimbal_Task(Can gimbals,rc_info_t* rc)
{
	GMY_SPEEDPID.fdb=GMYawEncoder.filter_rate;
	GMY_ANGELPID.fdb=GMYawEncoder.ecd_angle;
	GMP_ANGELPID.fdb=GMPitchEncoder.ecd_angle;
	GMP_SPEEDPID.fdb=GMPitchEncoder.filter_rate;
	GMY_ANGELPID.ref = 0;
	
			//fuzzy test
			//if(((GMY_ANGELPID.ref-GMY_ANGELPID.fdb)<5.0f)||((GMY_ANGELPID.ref-GMY_ANGELPID.fdb)>-5.0f))
			//{PID_Calc_Debug(&GMY_ANGELPID,1,0.000,0);}
			//else
				PID_Calc_Debug(&GMY_ANGELPID,0.5,0.05,10); // a debug version of PID_Calc for testing parameters (P=0.6,I=0.0003,D=8)
			GMY_SPEEDPID.ref = 0;
			//GMY_SPEEDPID.fdb = GMYawEncoder.filter_rate;
	     PID_Calc_Debug(&GMY_SPEEDPID,150.0,5.8,0.4);		
	    if(((GMY_SPEEDPID.ref-GMY_SPEEDPID.fdb)<15.0f)||((GMY_SPEEDPID.ref-GMY_SPEEDPID.fdb)>-15.0f))
			{
				PID_Calc_Debug(&GMY_SPEEDPID,50.0,5.8,6.4);
			}
	    
			
			GMP_ANGELPID.ref = 0;
			//GMP_ANGELPID.fdb = GMPitchEncoder.ecd_angle;			
			PID_Calc_Debug(&GMP_ANGELPID,0.8,0.05,10);
			GMP_SPEEDPID.ref =0 ;
			//GMP_SPEEDPID.fdb = GMPitchEncoder.filter_rate;
			PID_Calc_Debug(&GMP_SPEEDPID,100.0,0.8,0.9);
			if(((GMP_SPEEDPID.ref-GMP_SPEEDPID.fdb)<15.0f)||((GMP_SPEEDPID.ref-GMP_SPEEDPID.fdb)>-15.0f))
			{
				PID_Calc_Debug(&GMP_SPEEDPID,50.0,0.8,0.9);
			}
			//set_GM_speed(1000,1000);
			set_GM_speed(0,-GMP_SPEEDPID.output+GMP_ANGELPID.output);
	    off.yaw_offset=imu.yaw;
			off.rol_offset=imu.rol;
			
}

 //can self hold and point to a given Yaw.This version may induce severe bug!!!
void GM_task_stable(int pitref, int yawref, int mode) //0 cali, 1 rc
{
	GMY_ANGELPID.fdb = imu.yaw;
	GMY_SPEEDPID.fdb = imu.wz;
	if (mode==1)
		GMY_ANGELPID.ref = (int)(GMY_ANGELPID.ref+yawref)%360;// To be adjusted
	else
		GMY_ANGELPID.ref=0;
	PID_Calc_Debug(&GMY_ANGELPID,PID[VALUEY],0,PID[valuey]); // parameter waiting for adjust
	GMY_SPEEDPID.ref = 0;
		if(((GMY_SPEEDPID.ref-GMY_SPEEDPID.fdb)<15.0f)&&((GMY_SPEEDPID.ref-GMY_SPEEDPID.fdb)>-15.0f))
			{
				//GMY_SPEEDPID.ref =0 ;
				PID_Calc_Debug(&GMY_SPEEDPID,PID[VALUEY],0,PID[valuey]);
				OUTPUTY=(GMY_SPEEDPID.output*10+GMY_ANGELPID.output*3);
				//OUTPUTY=GMY_SPEEDPID.output*7;
			}
	else
	{
		PID_Calc_Debug(&GMY_SPEEDPID,PID[VALUE],0,PID[value]);
		OUTPUTY=(GMY_ANGELPID.output*10);
		//OUTPUTY=GMY_SPEEDPID.output*7;
	}


	//GMP_ANGELPID.fdb=fil.fil_pit;//low pass
	GMP_ANGELPID.fdb=imu.pit;
	GMP_SPEEDPID.fdb=imu.wy;
	if(mode==1) {
		if (GMP_ANGELPID.ref>45||GMP_ANGELPID.ref<-45)
			GMP_ANGELPID.ref += pitref;// Limit the max angle output To be adjusted
	}
	else
		GMP_ANGELPID.ref = 0;
	
	if(((GMP_ANGELPID.ref-GMP_ANGELPID.fdb)<2.0f)&&((GMP_ANGELPID.ref-GMP_ANGELPID.fdb)>-2.0f))
			{
				//PID_Calc_Debug(&GMP_ANGELPID,100.8,0,1.4);
				PID_Calc_Debug(&GMP_ANGELPID,PID[VALUE],0,PID[value+2]);
				GMP_SPEEDPID.ref =0;// parameter waiting for adjusting
	      //PID_Calc_Debug(&GMP_SPEEDPID,150,0,1.2); // parameter waiting for adjusting
				PID_Calc_Debug(&GMP_SPEEDPID,PID[VALUE+1],0,PID[value+3]);
				//set_GM_speed((-GMY_SPEEDPID.output+GMY_ANGELPID.output)*5,((GMP_SPEEDPID.output*7)));   // series connect pid trail
				OUTPUTP=(-GMP_ANGELPID.output*3+GMP_SPEEDPID.output*7)*1.8;
				//OUTPUTP=(GMP_SPEEDPID.output*2);  //Right
			}
	else
		{
		    //PID_Calc_Debug(&GMP_ANGELPID,100.8,0,0); 
		    PID_Calc_Debug(&GMP_ANGELPID,PID[VALUE],0,PID[value+2]); // parameter waiting for adjust
	      GMP_SPEEDPID.ref =0;
		    //PID_Calc_Debug(&GMP_SPEEDPID,50.0,0,0.9);
		    PID_Calc_Debug(&GMP_SPEEDPID,PID[VALUE+1],0,PID[value+3]);
	      OUTPUTP=GMP_ANGELPID.output*9;
				//set_GM_speed((-GMY_SPEEDPID.output+GMY_ANGELPID.output)*5,GMP_SPEEDPID.output*5);
		    //OUTPUTP=(GMP_SPEEDPID.output*2); //Right
	     	//OUTPUTP=(-GMP_ANGELPID.output*3+GMP_SPEEDPID.output*7)*1.8;
	    } 
	error_sample=(int)(GMP_ANGELPID.err[0]*1000);
  set_GM_speed(OUTPUTY,OUTPUTP);	// Can adjust weight
}		// Can adjust weight



/*



*/
//(-GMP_SPEEDPID.output+GMP_ANGELPID.output)*10
/* =========================================== */
//
//
//
//
//
//
/* Stuff below this line is the new experimental way to use CAN */
//
//
//
//
//
//
/* ========== smart CAN ========== */
Can** all_rx_devices;
unsigned int total_rx_device = 0;
/* =============================== */

/* ========== smart CAN initialize functions ========== */
void Device_Initialize(Can* device,const uint32_t StdId, const uint32_t IDE, const uint32_t RTR, const uint32_t DLC, const uint32_t rx_StdId, const int rx_buffer_size)
{
	device->StdId = StdId;
	device->IDE = IDE;
	device->RTR = RTR;
	device->DLC = DLC;
	device->rx_StdId = rx_StdId;
	device->canTxHeader.IDE = device->IDE;
	device->canTxHeader.StdId = device->StdId;
	device->canTxHeader.DLC = device->DLC;
	//device.canTxHeader.RTR = device.RTR;
	device->rx_buffer_size = rx_buffer_size;
	
	Device_ResizeRxBuffer(device, device->rx_buffer_size);
	Device_Activate_Rx(device);
}
void Device_ResizeRxBuffer(Can* device, int rx_buffer_size)
{
	uint8_t* new_rx_buffer = realloc(device->data, rx_buffer_size);
	if (new_rx_buffer == NULL)
	{
		// error check
	}
	else
	{
		device->data = new_rx_buffer;
	}
}
/* ==================================================== */

/* ========== smart CAN transmit functions ========== */
void Can_Transmit(Can* device,CAN_HandleTypeDef* hcan,uint8_t* canMsg)
{
	CAN_TxHeaderTypeDef msgHeader = device->canTxHeader;
	HAL_CAN_AddTxMessage(hcan,&msgHeader,canMsg,(void*)CAN_TX_MAILBOX0);
}
/* ================================================== */

/* ========== smart CAN receive functions ========== */
void Device_Activate_Rx(Can* device)
{
	Can** new_rx_device = realloc(all_rx_devices, ++total_rx_device);
	if (new_rx_device == NULL)
	{
		// error check
	}
	else
	{
		all_rx_devices = new_rx_device;
		all_rx_devices[total_rx_device-1] = device;
	}
}
void Device_Receive(CAN_RxHeaderTypeDef* canRxHeader,uint8_t* canRxMsg)
{
	for (int i = 0; i < total_rx_device; ++i)
	{
		if (canRxHeader->StdId == all_rx_devices[i]->rx_StdId)
		{
			for (int j = 0; j < all_rx_devices[i]->rx_buffer_size; ++j)
			{
				all_rx_devices[i]->data[j] = canRxMsg[j];
			}
		}
	}
}
void Can_Receive(CAN_HandleTypeDef* hcan, uint8_t* canRxMsg)	// place this in can1/2_rx_isr
{
	can_count++;
	CAN_RxHeaderTypeDef canRxHeader;
	HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&canRxHeader,canRxMsg);
	Device_Receive(&canRxHeader, canRxMsg);
}
/* ================================================= */

/* ========== smart CAN test functions ========== */
void test_smart_can(Can* device, CAN_HandleTypeDef* hcan,int16_t cm1_iq,int16_t cm2_iq,int16_t cm3_iq,int16_t cm4_iq)
{
		uint8_t canTxMsg0[8];
    canTxMsg0[0] = (uint8_t)(cm1_iq >> 8);
    canTxMsg0[1] = (uint8_t)cm1_iq;
    canTxMsg0[2] = (uint8_t)(cm2_iq >> 8);
    canTxMsg0[3] = (uint8_t)cm2_iq;
    canTxMsg0[4] = (uint8_t)(cm3_iq >> 8);
    canTxMsg0[5] = (uint8_t)cm3_iq;
    canTxMsg0[6] = (uint8_t)(cm4_iq >> 8);
    canTxMsg0[7] = (uint8_t)cm4_iq;
    Can_Transmit(device, hcan, canTxMsg0);
}
/* ============================================== */
//
//
//
//
//
//
/* Stuff below this line is for testing PID */
//
//
//
//
//
//
/* ========== PID ========== */
void PID_Calc(PID_Regulator_t *pid)
{
	pid->err[1] = pid->err[0];
	pid->err[0] = (pid->ref - pid->fdb);

	//calculate PID
	pid->KpComponent = pid->kp * pid->err[0];
	pid->KiComponent += pid->ki * pid->err[0];
	pid->KdComponent = pid->kd * (pid->err[0] - pid->err[1]);
	pid->output = pid->KpComponent + pid->KiComponent+ pid->KdComponent;
	
	//output value limit
	if((pid->output) > pid->output_limit)
		(pid->output>0) ? (pid->output=pid->output_limit) : (pid->output = -pid->output_limit);
}

void PID_Calc_Debug(PID_Regulator_t *pid,float kp,float ki,float kd)
{
	pid->err[1] = pid->err[0];
	pid->err[0] = (pid->ref - pid->fdb);

	//calculate PID
	pid->KpComponent = kp * pid->err[0];
	
	//if(pid->last_output > 750.0f/70.f && pid->last_output < 1500.0f/70.0f)
	//if(fabs(pid->ref-pid->fdb)<5)
	pid->KiComponent += ki * pid->err[0];
	//if(fabs(pid->KpComponent) < 1) 
		
	pid->KdComponent = kd * (pid->err[0] - pid->err[1]);
	pid->output = pid->KpComponent + pid->KiComponent+ pid->KdComponent;
	
	//if(fabs(pid->KiComponent)>16.5)
		//(pid->KiComponent>0) ? (pid->KiComponent=16.5) : (pid->KiComponent=-16.5);
	//output value limit
	if(((pid->output) > pid->output_limit)||((pid->output)<-(pid->output_limit)))
	{(pid->output>0) ? (pid->output=pid->output_limit) : (pid->output = -pid->output_limit);}
	pid->last_output = pid->output;
}

void EncoderProcess(volatile Encoder *v, uint8_t* msg)
{
	int i=0;
	int32_t temp_sum = 0;    
	v->last_raw_value = v->raw_value;
	v->raw_value = (msg[0]<<8)|msg[1];
	v->diff = v->raw_value - v->last_raw_value;
	if(can_count < 50)
	{
		v->ecd_raw_rate = 0;
	}
	else
	{
		if(v->diff < -7000)    
		{
			v->round_cnt++;
			v->ecd_raw_rate = v->diff + 8192;
		}
		else if(v->diff>7000)
		{
			v->round_cnt--;
			v->ecd_raw_rate = v->diff- 8192;
		}		
		else
		{
			v->ecd_raw_rate = v->diff;
		}
	}
	
	v->ecd_value = v->raw_value + v->round_cnt * 8192;
	
	//v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*360/8192 + v->round_cnt * 360;
	v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*360/8192.0f;
	v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
	if(v->buf_count == RATE_BUF_SIZE)
	{
		v->buf_count = 0;
	}
	
	for(i = 0;i < RATE_BUF_SIZE; i++)
	{
		temp_sum += v->rate_buf[i];
	}
	v->filter_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);		

}
void GetEncoderBias(volatile Encoder *v,CAN_RxHeaderTypeDef *rxHeader,uint8_t* msg)
{

            v->ecd_bias = (msg[0]<<8)|msg[1]; 
            v->ecd_value = v->ecd_bias;
            v->last_raw_value = v->ecd_bias;
            v->temp_count++;
}
void PID_Reset(PID_Regulator_t *pid)
{
	pid->ref = 0;
	pid->fdb = 0;
	pid->output = 0;
}
void send_Chassis_Msg(Can* chassis, int16_t cm1_iq,int16_t cm2_iq,int16_t cm3_iq,int16_t cm4_iq)
{
		uint8_t canTxMsg0[8];
    canTxMsg0[0] = (uint8_t)(cm1_iq >> 8);
    canTxMsg0[1] = (uint8_t)cm1_iq;
    canTxMsg0[2] = (uint8_t)(cm2_iq >> 8);
    canTxMsg0[3] = (uint8_t)cm2_iq;
    canTxMsg0[4] = (uint8_t)(cm3_iq >> 8);
    canTxMsg0[5] = (uint8_t)cm3_iq;
    canTxMsg0[6] = (uint8_t)(cm4_iq >> 8);
    canTxMsg0[7] = (uint8_t)cm4_iq;
    Can_Transmit(chassis, &hcan1, canTxMsg0);
}
void set_Chassis_Pid_Speed(Can chassis, rc_info_t* rc)
{
	HAL_Delay(1);
	CM1SpeedPID.ref =  (-rc->ch2*0.45*0.075 + rc->ch1*0.45*0.075 + rc->ch3*0.30*0.075)*18;
	CM2SpeedPID.ref = (rc->ch2*0.45*0.075 + rc->ch1*0.45*0.075 + rc->ch3*0.30*0.075)*18;
	CM3SpeedPID.ref = (rc->ch2*0.45*0.075 - rc->ch1*0.45*0.075 + rc->ch3*0.30*0.075)*18;
	CM4SpeedPID.ref = (-rc->ch2*0.45*0.075 - rc->ch1*0.45*0.075 + rc->ch3*0.30*0.075)*18; 
  CM1SpeedPID.fdb = CM1Encoder.filter_rate;
	CM2SpeedPID.fdb = CM2Encoder.filter_rate;
	CM3SpeedPID.fdb = CM3Encoder.filter_rate;
	CM4SpeedPID.fdb = CM4Encoder.filter_rate;
  PID_Calc(&CM1SpeedPID);
	PID_Calc(&CM2SpeedPID);
	PID_Calc(&CM3SpeedPID);
	PID_Calc(&CM4SpeedPID);
	set_CM_speed((CM1SpeedPID.output*SPEED_OUTPUT_ATTENUATION),CM2SpeedPID.output*SPEED_OUTPUT_ATTENUATION,CM3SpeedPID.output*SPEED_OUTPUT_ATTENUATION,(CM4SpeedPID.output*SPEED_OUTPUT_ATTENUATION));
	if(rc->sw1==2 && rc->sw2==2)
	{
	   CM1SpeedPID.ref = 0;
	   CM2SpeedPID.ref = 0;
	   CM3SpeedPID.ref = 0;
	   CM4SpeedPID.ref = 0;
	}
}

void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(hcan->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();
  
    /**CAN1 GPIO Configuration    
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }

}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* hcan)
{
  if(hcan->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();
  
    /**CAN1 GPIO Configuration    
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX 
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

    /* CAN1 interrupt DeInit */
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }

}


