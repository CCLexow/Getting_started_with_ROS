/*
 * cpp_main.cpp
 *
 *  Created on: 30.10.2018
 *      Author: Carl Christian Lexow
 */

#include <cpp_main.h>
#include "main.h"
#include "stm32f1xx_hal.h"
#include "ringbuffer.h"
#include "ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "nbt.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"


extern uint8_t RxBuffer[RxBufferSize];
struct ringbuffer rb;

ros::NodeHandle nh;
std_msgs::String str_msg;

sensor_msgs::LaserScan xmsg_LaserScan;

extern "C" void messageCb(const geometry_msgs::Twist& msg);
ros::Publisher chatter("version", &str_msg);
ros::Subscriber<geometry_msgs::Twist> sub("twist_cmd", messageCb);

ros::Publisher xpbIR_Scan("IR_Scan", &xmsg_LaserScan);

static nbt_t publish_nbt;
static nbt_t ros_nbt;
static nbt_t pub_IR_Scan;

ros::Time xtSysTime;

#define IR_DBG_SCAN_LENGTH	320
const uint16_t cau16ScanNoObstacle[IR_DBG_SCAN_LENGTH]={
		279, 279, 279, 279, 279, 279, 279, 279, 283, 284, 284, 284, 284, 287, 288, 288, 289, 298, 297, 297, 304, 304, 304, 307, 309, 309, 317, 317, 317, 328, 328, 337, 337, 347, 351, 339, 328, 318, 313, 309, 304, 297, 291, 289, 284, 279, 273, 273, 267, 259, 256, 256, 252, 247, 242, 242, 237, 235, 234, 229, 228, 223, 224, 219, 219, 216, 216, 214, 212, 212, 212, 209, 209, 209, 208, 206, 206, 206, 206, 206, 206, 206, 206, 206, 206, 206, 209, 209, 209, 209, 209, 213, 212, 212, 216, 216, 217, 219, 219, 224, 223, 229, 233, 233, 237, 238, 242, 248, 251, 252, 255, 259, 267, 268, 274, 279, 284, 287, 289, 297, 304, 309, 308, 317, 317, 328, 346, 352, 357, 347, 337, 337, 328, 328, 328, 318, 318, 313, 309, 308, 305, 304, 304, 297, 298, 298, 289, 289, 289, 287, 283, 284, 284, 284, 284, 283, 279, 279, 279, 279, 279, 278, 278, 279, 279, 279, 280, 284, 284, 284, 284, 284, 285, 289, 289, 288, 288, 297, 297, 298, 304, 304, 307, 309, 309, 317, 317, 317, 319, 328, 327, 327, 328, 338, 338, 328, 322, 318, 309, 309, 304, 297, 289, 288, 284, 279, 274, 267, 259, 256, 256, 252, 248, 242, 242, 238, 234, 232, 229, 224, 223, 219, 219, 216, 216, 216, 212, 212, 209, 209, 208, 206, 206, 206, 204, 205, 202, 202, 202, 202, 202, 202, 202, 202, 203, 203, 203, 203, 206, 206, 206, 206, 209, 209, 209, 212, 212, 216, 216, 219, 219, 224, 224, 229, 231, 234, 237, 242, 242, 248, 252, 253, 256, 259, 268, 274, 279, 284, 284, 289, 297, 304, 305, 309, 317, 317, 328, 347, 355, 347, 345, 338, 338, 328, 328, 318, 318, 318, 309, 309, 309, 304, 304, 302, 298, 298, 289, 289, 289, 289, 284, 284, 284, 284, 284, 279, 279, 279, 279, 279
};

const uint16_t cau16ScanObstacle[IR_DBG_SCAN_LENGTH]={
		279, 279, 279, 279, 279, 281, 284, 284, 284, 284, 284, 289, 289, 289, 297, 297, 297, 303, 304, 304, 309, 309, 317, 317, 317, 328, 328, 327, 337, 337, 347, 357, 348, 338, 319, 318, 309, 309, 304, 297, 289, 284, 283, 279, 274, 268, 260, 256, 256, 252, 248, 243, 241, 237, 233, 233, 229, 224, 224, 219, 219, 216, 216, 213, 213, 209, 209, 209, 206, 206, 206, 205, 202, 202, 202, 202, 202, 202, 202, 202, 202, 203, 203, 203, 203, 203, 203, 206, 206, 206, 209, 209, 209, 212, 212, 212, 212, 206, 175, 154, 146, 145, 144, 144, 144, 144, 144, 146, 146, 146, 146, 147, 147, 149, 149, 151, 151, 153, 154, 156, 158, 158, 159, 162, 164, 164, 166, 168, 171, 174, 178, 179, 182, 182, 172, 124, 96, 89, 90, 93, 98, 101, 107, 112, 116, 121, 127, 136, 153, 192, 252, 274, 279, 279, 279, 279, 279, 279, 279, 279, 279, 279, 278, 279, 279, 279, 281, 284, 284, 284, 283, 288, 289, 289, 289, 297, 297, 297, 304, 304, 306, 309, 309, 311, 314, 309, 289, 229, 178, 158, 153, 150, 148, 147, 147, 147, 149, 151, 157, 184, 247, 279, 284, 279, 279, 274, 267, 259, 259, 255, 252, 248, 248, 242, 237, 237, 233, 228, 229, 224, 224, 219, 219, 216, 216, 216, 212, 212, 210, 209, 209, 209, 209, 206, 206, 206, 206, 206, 206, 206, 206, 206, 206, 206, 206, 206, 208, 209, 209, 209, 212, 212, 212, 216, 216, 217, 219, 219, 224, 224, 229, 233, 233, 237, 237, 242, 248, 250, 252, 256, 259, 260, 267, 274, 279, 280, 284, 288, 297, 304, 304, 309, 317, 327, 337, 347, 347, 338, 338, 327, 319, 317, 317, 309, 309, 309, 304, 304, 304, 297, 297, 298, 289, 289, 289, 288, 284, 283, 284, 284, 282, 279, 279, 279, 279, 279, 279, 279, 279, 279
};

const int32_t ci32AngleMin = 0;
const int32_t ci32AngleMax = 3588750;
const int32_t ci32AngleIncrement = 11250;

const float cfltAngleMin = 0;
const float cfltAngleMax = 2 * M_PI * ((float)ci32AngleMax) / 360;
const float cfltAngleIncrement = 2 * M_PI * ((float)ci32AngleIncrement) / 3600000;

#define IR_SCAN_DATA_LENGTH	40
float afltScanData[IR_SCAN_DATA_LENGTH];
uint16_t au16ScanData[IR_SCAN_DATA_LENGTH];



const int32_t ci32ANGLE_NOT_SET = -444444;
int32_t i32StartAngle = ci32ANGLE_NOT_SET;
int32_t i32StopAngle = ci32ANGLE_NOT_SET;

extern "C" int32_t i32Put_IR_Scan_Data(int32_t i32Angle, uint16_t u16Distance)
{
	static uint8_t su08WriteIdx = 0;
	int32_t i32RetVal;

	/* verify that there is still space left within the buffer */
	if(IR_SCAN_DATA_LENGTH > su08WriteIdx)
	{
		/* save data set */
		au16ScanData[su08WriteIdx] = u16Distance;
		/* start angle already set ? */
		if(ci32ANGLE_NOT_SET == i32StartAngle)
		{
			/* save */
			i32StartAngle = i32Angle;
		}
	}
	else
	{
		/* error */
		i32RetVal =  -1;
	}
	/* count data set */
	su08WriteIdx++;
	/* buffer full ? */
	if(IR_SCAN_DATA_LENGTH == su08WriteIdx)
	{
		/* take last angle as stop angle */
		i32StopAngle = i32Angle;
		/* signal that buffer is full */
		i32RetVal =  0;
		/* reset write idx */
		su08WriteIdx = 0;
	}
	else
	{
		/* still space available */
		i32RetVal =  1;
	}
	return i32RetVal;
}

extern "C" void Flush_IR_Scan_Data(void)
{
	/* delete buffer content */
	for(uint8_t u08Idx=0; u08Idx<IR_SCAN_DATA_LENGTH; u08Idx++)
	{
		au16ScanData[u08Idx] = 0;
	}
	/* preset angles */
	i32StartAngle = ci32ANGLE_NOT_SET;
	i32StopAngle = ci32ANGLE_NOT_SET;
}



extern "C" void messageCb(const geometry_msgs::Twist& msg)
{
		//Fill subscriber
}

extern "C" void cdc_receive_put(uint8_t value)
{
	ringbuffer_putchar(&rb, value);
}

extern "C" void init_ROS()
{

	Flush_IR_Scan_Data();

	ringbuffer_init(&rb, RxBuffer, RxBufferSize);

	// Initialize ROS
	nh.initNode();

	nh.advertise(chatter);
	nh.advertise(xpbIR_Scan);
	nh.subscribe(sub);

	NBT_init(&publish_nbt, 500);
	NBT_init(&ros_nbt, 10);
	NBT_init(&pub_IR_Scan, 100);

}

extern "C" void version_handler()
{
	  if (NBT_handler(&publish_nbt))
	  {
		  char chrVersion[20];
		  for(uint8_t u08Idx=0; u08Idx<strlen(VERSION); u08Idx++)
		  {
			  chrVersion[u08Idx] = VERSION[u08Idx];
		  }
		  char version[] = "Calle ist einfach genial !!!";
		  str_msg.data = version;
		  //str_msg.data = printf("Version: %s",(const char *)&chrVersion[0]);
		  //sprintf((char*)&str_msg.data,"Version: %s", VERSION);
		  chatter.publish(&str_msg);

	  }
}

extern "C" void IR_Scan_handler(void)
{
	static uint32_t su32MsgIdx = 0;
	static uint8_t su08Switch = 0;
	static uint16_t su16ReadIdx = 0;
	static int32_t si32Angle = 0;
	uint8_t u08SendMessage = 0;
	volatile int32_t i32RetVal;

	  if (NBT_handler(&pub_IR_Scan))
	  {

		  /* debug: read stored data set */
		  if(0 == su08Switch)
		  {
			  /* put data set (without obstacle) */
			  i32RetVal = i32Put_IR_Scan_Data(si32Angle, cau16ScanNoObstacle[su16ReadIdx]);
		  }
		  else
		  {
			  /* put data set (with obstacle) */
			  i32RetVal = i32Put_IR_Scan_Data(si32Angle, cau16ScanObstacle[su16ReadIdx]);
		  }

		  /* count data set */
		  su16ReadIdx++;
		  /* and sum up angle */
		  si32Angle += ci32AngleIncrement;
		  /* evaluate return value */
		  if(0 == i32RetVal)
		  {
			  /* buffer is full and ready for sending */
			  u08SendMessage = 1;

			  /* continue or switch? */
			  if((IR_DBG_SCAN_LENGTH) == su16ReadIdx)
			  {
				  /* preset angle and read idx for next run */
				  si32Angle = 0;
				  su16ReadIdx = 0;
				  /* choose switch direction */
				  if(0 == su08Switch)
				  {
					  /* switch to obstacle data set on next call */
					  su08Switch = 1;
				  }
				  else
				  {
					  /* switch to no obstacle data set on next call */
					  su08Switch = 0;
				  }
			  }
		  }

		  /* check if message should be sent */
		  if(1 == u08SendMessage)
		  {
			  /* prepare distance scan data */
			  for(uint8_t u08Idx=0; u08Idx < IR_SCAN_DATA_LENGTH; u08Idx++)
			  {
				  afltScanData[u08Idx] = ((float)au16ScanData[u08Idx])/1000;
			  }

			  xtSysTime.sec = su32MsgIdx;
			  /* set values */
			  xmsg_LaserScan.angle_min = 2 * M_PI * ((float)i32StartAngle) / 3600000;
			  xmsg_LaserScan.angle_max = 2 * M_PI * ((float)i32StopAngle) / 3600000;
			  xmsg_LaserScan.angle_increment = cfltAngleIncrement;
			  xmsg_LaserScan.time_increment = 4.0/40;
			  xmsg_LaserScan.scan_time = 4.0;
			  xmsg_LaserScan.range_min  = 7.0 / 100;
			  xmsg_LaserScan.range_max = 55.0 / 100;
			  xmsg_LaserScan.ranges = &afltScanData[0];
			  xmsg_LaserScan.ranges_length = IR_SCAN_DATA_LENGTH;
			  xmsg_LaserScan.header.frame_id = "IR_Scan_Map";
			  xmsg_LaserScan.header.seq = su32MsgIdx++;
			  //xmsg_LaserScan.header.stamp = ros::Time::now();
			  //xmsg_LaserScan.header.stamp = HAL_GetTick();
			  xmsg_LaserScan.header.stamp = xtSysTime;

			  /* publish message */
			  xpbIR_Scan.publish(&xmsg_LaserScan);

			  /* prepare next scan */
			  Flush_IR_Scan_Data();
		  }
	  }
}

extern "C" void spinOnce()
{
	  if (NBT_handler(&ros_nbt))
	  {
		nh.spinOnce();
	  }
}

