/* drivers/input/keyboard/lge_touch_driver.c
 *
 * Copyright (C) 2011 LGE. 
 * 
 * Writer: yehan.ahn@lge.com
 *
 * Most lines are copied from touch_driver of STAR-P999.
 * That is written by joseph.jung@lge.com and taewan.kim@lge.com.
 * Please check the STAR-P999 or contact writers if you want more information about original source code.
 *
 * This file is used by LGE_touch_deriver.c.
 * it is related with specific touch-device.
 * You should check the document or contact the writer in order to understand whole process.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/kthread.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <mach/gpio.h>
#include <linux/semaphore.h>
#include <linux/irq.h>
#include <linux/freezer.h>

#include <linux/lge_touch_synaptics.h>
#include <linux/lge_touch_driver.h>

#include <linux/bd6084_bl.h>
//                                                   
#if defined (CONFIG_SU880) || defined (CONFIG_KU8800) || defined (CONFIG_LU8800) || defined (CONFIG_KS1103)
#include <../gpio-names.h>
#endif

//                                                    
#if defined (SYNAPTICS_TOUCH_FW_UPGRADE)
#include <linux/syscalls.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#endif
//                                                    

//#define STAR_MELT_SUPPORT
#define STAR_FW_UPGRADE
#define STAR_TOUCH_GRIP_SUPPRESSION
/* 
                           
                                                                 
 */
#define LGE_NOMELT
 
#define GET_BIT_MASK(_finger_state_reg)	\
		(_finger_state_reg[2] & 0x04)<<7 | (_finger_state_reg[2] & 0x01)<<8 |	\
		(_finger_state_reg[1] & 0x40)<<1 | (_finger_state_reg[1] & 0x10)<<2 |(_finger_state_reg[1] & 0x04)<<3 | (_finger_state_reg[1] & 0x01)<<4 |	\
		(_finger_state_reg[0] & 0x40)>>3 | (_finger_state_reg[0] & 0x10)>>2 |(_finger_state_reg[0] & 0x04)>>1 | (_finger_state_reg[0] & 0x01)

#define GET_INDEX_FROM_MASK(_index, _bit_mask)	\
for(; !((_bit_mask>>_index)&0x01) && _index <= SYNAPTICS_FINGER_MAX; _index++);	\
if(_index <= SYNAPTICS_FINGER_MAX) _bit_mask &= ~(_bit_mask & (1<<(_index)));


#define TS_SNTS_GET_X_POSITION(_high_reg, _low_reg) \
		( ((u16)((_high_reg << 4) & 0x000007F0) | (u16)(_low_reg&0x0F)) * (LGE_TOUCH_RESOLUTION_X - 1) / 1036)
#define TS_SNTS_GET_Y_POSITION(_high_reg, _low_reg) \
		( ((u16)((_high_reg << 4) & 0x000007F0) | (u16)((_low_reg >> 4) & 0x0F)) * (LGE_TOUCH_RESOLUTION_Y - 1) / 1728)
#define TS_SNTS_GET_WIDTH(_width) \
		((((_width & 0xf0) >> 4) - (_width & 0x0f)) > 0)? (_width & 0xf0) >> 4 : _width & 0x0f
#define TS_SNTS_GET_PRESSURE(_pressure) \
		_pressure

#if defined (LGE_NOMELT)
#define TS_SNTS_GET_LOWDATA_X_POSITION(_high_reg, _low_reg) \
		((u16)((_high_reg << 4) & 0x000007F0) | (u16)(_low_reg&0x0F))
#define TS_SNTS_GET_LOWDATA_Y_POSITION(_high_reg, _low_reg) \
		((u16)((_high_reg << 4) & 0x000007F0) | (u16)((_low_reg >> 4) & 0x0F))
#endif

enum {X_HIGH_POSITION=0, Y_HIGH_POSITION, XY_LOW_POSITION, XY_WIDTH, PRESSURE};

#define ADJUST_LEVEL 5
#define SQUARE(x)		((x) * (x))

const u16 ADJUST_FACTOR_LEVEL[ADJUST_LEVEL] = {8,6,4,2,1};
const u16 ADJUST_BASIS_LEVEL[ADJUST_LEVEL] = {5,11,19,29,40};
const u16 ADJUST_FACTOR_BASE = 4;

#if defined (CONFIG_SU880) || defined (CONFIG_KU8800) || defined (CONFIG_LU8800) || defined (CONFIG_KS1103)    
#define IS_MENU(_x)			(_x > 15  && _x < 145)
#define IS_HOME(_x)			(_x > 175 && _x < 305)
#define IS_BACK(_x)			(_x > 335 && _x < 465)
#define IS_SEARCH(_x)		(_x < 0)
#define IS_PANEL(_y)		(_y >= 0  && _y <= 800)
#else
#define IS_MENU(_x)			(_x > 15  && _x < 115)
#define IS_HOME(_x)			(_x > 145 && _x < 225)
#define IS_BACK(_x)			(_x > 255 && _x < 335)
#define IS_SEARCH(_x)		(_x > 365 && _x < 465)
#define IS_PANEL(_y)		(_y >= 0  && _y <= 800)
#endif
struct synaptics_ts_data {
	struct touch_device_caps caps;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct semaphore sem;
	struct task_struct*	task;
	u32 gpio;
	u32 irq_gpio;
	u32 flags;
	int (*power)(char* reg_id, bool on);
	u32 touch;
//                                                    
#if defined (SYNAPTICS_TOUCH_FW_VERSION)
	char fw_rev;
#endif
//                                                    
//                                                    
#if defined (SYNAPTICS_TOUCH_FW_UPGRADE)
	char fw_path[256];
#endif
//                                                    
};

static const struct touch_device_caps Synaptics_Capabilities =
{
	1,	//IsMultiTouchSupported
	1, 	//isButtonSupported
	10,	//MaxNumberOfFingerCoordReported;
	0,	//IsRelativeDataSupported
	0,	//MaxNumberOfRelativeCoordReported
	15,	//MaxNumberOfWidthReported
	0xFF,	//MaxNumberOfPressureReported
	0,	// Gesture
	0,	//IsWidthSupported
	0,	//IsPressureSupported, mandatory for multi-touch
	0,	//IsFingersSupported
	0,	//XMinPosition
	0,	//YMinPosition
	LGE_TOUCH_RESOLUTION_X-1,	//XMaxPosition
	LGE_TOUCH_RESOLUTION_Y-1,	//YMaxPosition
	0,
};

struct ts_sensor_data
{
	u8 device_status_reg;						//0x13
	u8 interrupt_status_reg;					//0x14
	u8 finger_state_reg[3];						//0x15~0x17

	u8 fingers_data[SYNAPTICS_FINGER_MAX][5];				//0x18 ~ 0x49

	u8 gesture_flag0;							//0x4A
	u8 gesture_flag1;							//0x4B
	u8 pinch_motion_X_flick_distance;			//0x4C
	u8 rotation_motion_Y_flick_distance;		//0x4D
	u8 finger_separation_flick_time;			//0x4E
};

//                                                    
#if defined (SYNAPTICS_TOUCH_FW_UPGRADE)
struct synaptics_fw_data {
	unsigned char *image_bin;
	unsigned long image_size;
};
#endif
//                                                    

#if defined (LGE_NOMELT)
static char mode=1,numfinger=0, reportcnt=0, tapcount=0;
static int current_data_x, current_data_y, prex, prey, firstx, firsty;
#endif

static struct ts_sensor_data ts_reg_data={0};

#ifdef STAR_FW_UPGRADE
#if defined(CONFIG_KS1001) || defined (CONFIG_LU6500)
#include "synaptics_ts_firmware_TM1765.h"
#else if defined (CONFIG_SU880) || defined (CONFIG_KU8800) || defined (CONFIG_LU8800) || defined (CONFIG_KS1103)
#include "synaptics_ts_firmware_TM1962.h"
#endif

static int synaptics_ts_fw_upgrade(struct synaptics_ts_data* hTouch);
#endif

//                                                  
#if 1
static bool synaptics_ts_hw_reset(struct synaptics_ts_data* hTouch);
#else
static bool synaptics_ts_hw_reset(struct synaptics_ts_data* hTouch, u8 device_status_reg);
#endif
//                                                  

#if defined (LGE_NOMELT)
static void Synaptics_SetNoMeltMode (void* h_dev, bool binit);
#endif /*            */

#ifdef STAR_TOUCH_GRIP_SUPPRESSION
#define READ_BUFFER_LENGTH 20
u8 touch_grip_suppression_value = 0;
static bool touch_reset_flag = false;

#define IGNORE_IF_POSITION_IS_SUPPRESSION_AREA(_x)	\
if(_x < touch_grip_suppression_value || _x > STAR_TOUCH_RESOLUTION_X - touch_grip_suppression_value)	\
	continue;

ssize_t touch_gripsuppression_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	sprintf(buf, "%d\n", touch_grip_suppression_value);	
	return (ssize_t)(strlen(buf)+1);
}

ssize_t touch_gripsuppression_store(struct device *dev, struct device_attribute *attr, const char *buffer, size_t count)
{
	u8 *input;
	u32 value;

	count = count > READ_BUFFER_LENGTH ? READ_BUFFER_LENGTH : count;
	input = kzalloc(((int)count+1), GFP_KERNEL);
	if (!input) {
		return 0;
	}

	memcpy(input, buffer, count);

	input[count] = '\0';
	value = simple_strtoul(&input[0], '\0', 10);

	touch_grip_suppression_value = value;

	kfree(input);
	return count;
}

DEVICE_ATTR(gripsuppression, 0666, touch_gripsuppression_show, touch_gripsuppression_store);
#endif /*                                    */

//                                                    
#if defined (SYNAPTICS_TOUCH_FW_UPGRADE)
ssize_t touch_fw_upgrade_store(struct device *dev, struct device_attribute *attr, const char *buffer, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lge_touch_driver *td = i2c_get_clientdata(client);
	struct synaptics_ts_data *ts = (struct synaptics_ts_data*)(td->h_touch);
	int value = 0;
	int repeat = 0;

	sscanf(buffer, "%d %s", &value, ts->fw_path);

	printk(KERN_INFO "\n");
	if(ts->fw_path[0] != 0)
		DEBUG_MSG(E, "Firmware image path: %s\n", ts->fw_path[0] != 0 ? ts->fw_path : "Internal");

	for(repeat = 0; repeat < value; repeat++) {
		msleep(1000);
		printk(KERN_INFO "\n");
		DEBUG_MSG(E, "Firmware image upgrade: No.%d", repeat+1);		
		disable_irq_nosync(ts->irq_gpio);
		synaptics_ts_fw_upgrade(ts);
		enable_irq(ts->irq_gpio);
	}

	memset(ts->fw_path, 0x0, sizeof(ts->fw_path));

	return count;

}
DEVICE_ATTR(fw_upgrade, 0666, NULL, touch_fw_upgrade_store);
#endif
//                                                    

//                                                    
#if defined (SYNAPTICS_TOUCH_FW_VERSION)
static ssize_t show_fw_revision(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lge_touch_driver *td = i2c_get_clientdata(client);
	struct synaptics_ts_data *ts = (struct synaptics_ts_data*)(td->h_touch);
	
	int ret = 0;

	ret = sprintf(buf, "%d\n", ts->fw_rev);

	return ret;
}

DEVICE_ATTR(fw_ver, S_IRUGO | S_IWUSR, show_fw_revision, NULL);
#endif
//                                                    


static void tegra_touch_adjust_position(const u16 x_value, const u16 y_value, u16 *adjust_X, u16 *adjust_Y)
{
	u16 distant = int_sqrt(SQUARE(*adjust_X - x_value) + SQUARE(*adjust_Y - y_value));
	u16 i;

	for(i=0; i<ADJUST_LEVEL; i++){
		if(distant <= ADJUST_BASIS_LEVEL[i]){
			*adjust_X = (x_value * ADJUST_FACTOR_LEVEL[i] + *adjust_X * ADJUST_FACTOR_BASE) 
						/ (ADJUST_FACTOR_LEVEL[i] + ADJUST_FACTOR_BASE);
			*adjust_Y = (y_value * ADJUST_FACTOR_LEVEL[i] + *adjust_Y * ADJUST_FACTOR_BASE) 
						/ (ADJUST_FACTOR_LEVEL[i] + ADJUST_FACTOR_BASE);
			break;
		}
	}
	
	if(*adjust_Y < LGE_TOUCH_RESOLUTION_Y+20 && *adjust_Y >= LGE_TOUCH_RESOLUTION_Y)
		*adjust_Y = LGE_TOUCH_RESOLUTION_Y-1;

}

void device_close (void* h_dev)
{
    struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;

    kfree(hTouch);
}

bool device_open(void** h_touch, struct i2c_client *client, u32 touch)
{
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)0;
	struct lge_synaptics_platform_data *pdata;

	DO_A(hTouch = kzalloc(sizeof(struct synaptics_ts_data), GFP_KERNEL),  err_Device_Open);

    memset(hTouch, 0, sizeof(struct synaptics_ts_data));

	DO_F(i2c_check_functionality(client->adapter, I2C_FUNC_I2C), err_Device_Open);
	hTouch->client = client;

	DO_A(pdata = client->dev.platform_data, err_Device_Open);

	hTouch->flags = pdata->irqflags;
	hTouch->power = pdata->power;
	hTouch->gpio = pdata->gpio;
	hTouch->irq_gpio  = client->irq;
	hTouch->touch = touch;
	
	memcpy(&hTouch->caps, &Synaptics_Capabilities, sizeof(struct touch_device_caps));

#ifdef STAR_TOUCH_GRIP_SUPPRESSION
	DO_C(device_create_file(&client->dev, &dev_attr_gripsuppression) != 0, err_Device_Open);
#endif	
//                                                    
#if defined (SYNAPTICS_TOUCH_FW_UPGRADE)
	DO_C(device_create_file(&client->dev, &dev_attr_fw_upgrade) != 0, err_Device_Open);
#endif
//                                                    
//                                                    
#if defined (SYNAPTICS_TOUCH_FW_VERSION)
	DO_C(device_create_file(&client->dev, &dev_attr_fw_ver) != 0, err_Device_Open);
#endif
//                                                    

	*h_touch = (void *)hTouch;
  	return true;

err_Device_Open:
	device_close((void* )hTouch);
	return false;
} 

bool inputDevOpen(void* h_dev)
{
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;
	struct input_dev* input_dev;
	
	DO_A(hTouch->input_dev = input_allocate_device(), err_inputDevOpen2);
	input_dev = hTouch->input_dev;
	input_dev->name = LGE_TOUCH_NAME;

	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(KEY_MENU, input_dev->keybit);
	set_bit(KEY_BACK, input_dev->keybit);
	set_bit(KEY_HOME, input_dev->keybit);
	set_bit(KEY_SEARCH, input_dev->keybit);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, hTouch->caps.XMinPosition, hTouch->caps.XMaxPosition, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, hTouch->caps.YMinPosition, hTouch->caps.YMaxPosition, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, hTouch->caps.MaxNumberOfPressureReported , 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, hTouch->caps.MaxNumberOfWidthReported, 0, 0);
#if defined(CONFIG_SU880) || defined(CONFIG_KU8800) || defined(CONFIG_KS1103)  //                                        
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, SYNAPTICS_FINGER_MAX-1, 0, 0);
#endif
	DO_F(!input_register_device(input_dev), err_inputDevOpen);
	return true;

err_inputDevOpen:
	input_unregister_device(hTouch->input_dev);
err_inputDevOpen2:
	input_free_device(hTouch->input_dev);
	return false;
}

void inputDevClose(void* h_dev)
{
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;
	
	input_unregister_device(hTouch->input_dev);
	input_free_device(hTouch->input_dev);
}

bool inputDevSendABS(void* h_dev, struct touch_data data, u8 isPress)
{
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;

	DO_A(hTouch->input_dev, err_inputDevSendABS);
	if(isPress){
		input_report_abs(hTouch->input_dev, ABS_MT_POSITION_X, data.X_position);
		input_report_abs(hTouch->input_dev, ABS_MT_POSITION_Y, data.Y_position);
		input_report_abs(hTouch->input_dev, ABS_MT_TOUCH_MAJOR, data.pressure);
		input_report_abs(hTouch->input_dev, ABS_MT_WIDTH_MAJOR, data.width);
		// input_report_key(hTouch->input_dev, BTN_TOUCH, isPress); //for ICS
#if defined(CONFIG_SU880) || defined(CONFIG_KU8800) || defined(CONFIG_KS1103) //                                        
		input_report_abs(hTouch->input_dev, ABS_MT_TRACKING_ID, data.id);
#endif
	}
	input_mt_sync(hTouch->input_dev);
	
	return true;

err_inputDevSendABS:
	return false;
}

bool inputDevSendABSMulti(void* h_dev, struct touch_data* data, u8 total_num)
{
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;
	int i=0, check=0;
	
	DO_A(hTouch->input_dev, err_inputDevSendABSMulti);

	for(i=0; i<total_num; i++){
		if(IS_PANEL(data[i].Y_position)){
			input_report_abs(hTouch->input_dev, ABS_MT_POSITION_X, data[i].X_position);
			input_report_abs(hTouch->input_dev, ABS_MT_POSITION_Y, data[i].Y_position);
			input_report_abs(hTouch->input_dev, ABS_MT_TOUCH_MAJOR, data[i].pressure);
			input_report_abs(hTouch->input_dev, ABS_MT_WIDTH_MAJOR, data[i].width);
#if defined(CONFIG_SU880) || defined(CONFIG_KU8800) || defined(CONFIG_KS1103) //                                        
			input_report_abs(hTouch->input_dev, ABS_MT_TRACKING_ID, data[i].id);
#endif
			input_mt_sync(hTouch->input_dev);
			DEBUG_MSG(M, "[TOUCH] X[%d], Y[%d]\n", (int)data[i].X_position, (int)data[i].Y_position);
			check++;
		}
	}

	if(!check){
		DEBUG_MSG(M, "[TOUCH] mt_sync. \n");
		input_mt_sync(hTouch->input_dev);
	}
	
	return true;

err_inputDevSendABSMulti:
	return false;
}


bool inputDevSendButton(void* h_dev, u32 type, bool isPress)
{
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;

	DO_A(hTouch->input_dev, err_inputDevSendButton);
	input_report_key(hTouch->input_dev, type, isPress);
	return true;

err_inputDevSendButton:
	return false;
}

bool inputDevSendSync(void* h_dev, u8 state)
{
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;

	DO_A(hTouch->input_dev, err_inputDevSendSync);

	if(state != TOUCH_LOCK){
		input_sync(hTouch->input_dev);
		DEBUG_MSG(M, "[TOUCH] sync.\n");
	}

	return true;

err_inputDevSendSync:
	return false;
}

void init_touch_inputdev(void* h_dev, struct touch_inputdev_func* h_inputDev)
{
	h_inputDev->open = inputDevOpen;
	h_inputDev->close = inputDevClose;
	h_inputDev->send_ABS = inputDevSendABS;
	h_inputDev->send_ABS_Multi = inputDevSendABSMulti;
	h_inputDev->send_Button = inputDevSendButton;
	h_inputDev->send_Sync = inputDevSendSync;
}

bool taskOpen(void* h_dev)
{
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;
	
	DO_A(hTouch->task = kthread_create(task_handler, (void*)hTouch->touch, "LGE_touch_thread"), err_taskOpen);
	return true;

err_taskOpen:
	return false;	
}

void taskClose(void* h_dev)
{

}

bool taskStart(void* h_dev)
{
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;

	DO_A(hTouch->task, err_taskStart);
	wake_up_process( hTouch->task );
	return true;
	
err_taskStart:
	return false;
}

bool taskStop(void* h_dev)
{
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;

	DO_A(hTouch->task, err_taskStop);
	kthread_stop(hTouch->task);
	return true;

err_taskStop:
	return false;
}

void init_touch_task(void* h_dev, struct touch_task_func* h_task)
{
	h_task->open = taskOpen;
	h_task->close = taskClose;
	h_task->start = taskStart;
	h_task->stop = taskStop;
}

void interruptClose(void* h_dev)
{
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;

	free_irq(hTouch->irq_gpio, (void*)hTouch->touch);
}

bool interruptOpen(void* h_dev)
{
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;

	//init_MUTEX_LOCKED(&hTouch->sem);
	sema_init(&hTouch->sem, 0);
	DO_F(!request_irq(hTouch->irq_gpio, interrupt_handler, hTouch->flags, LGE_TOUCH_NAME, (void*)hTouch->touch), err_interruptOpen);

	return true;

err_interruptOpen:
	interruptClose((void*)hTouch);
	return false;
}

bool interruptEnable(void* h_dev)
{
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;
	DO_C(!hTouch->irq_gpio, err_interruptEnable);
	enable_irq(hTouch->irq_gpio);
	return true;
	
err_interruptEnable:
	return false;
}

bool interruptDisable(void* h_dev)
{
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;
	DO_C(!hTouch->irq_gpio, err_interruptDisable);
	disable_irq_nosync(hTouch->irq_gpio);
	return true;
	
err_interruptDisable:
	return false;
}

bool interruptStart(void* h_dev)
{
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;
	up( &hTouch->sem );
	
	return true;
}

bool interruptWait(void* h_dev)
{
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;
	int ret;
	do{
       ret = down_interruptible(&hTouch->sem);
       if (ret && !try_to_freeze())
            schedule();
    } while (ret);
	return true;
}

void init_touch_interrupt(void* h_dev, struct touch_interrupt_func* h_interrupt)
{
	h_interrupt->open = interruptOpen;
	h_interrupt->close = interruptClose;
	h_interrupt->enable = interruptEnable;
	h_interrupt->disable = interruptDisable;
	h_interrupt->start = interruptStart;
	h_interrupt->wait = interruptWait;
}

#define WAIT_TOUCH_POWER_READY(_client, _num)				\
{															\
	int retry = _num;										\
	while (retry-- > 0) {									\
		int ret = i2c_smbus_read_byte_data(_client, 0xb8);	\
		if (ret >= 0)										\
			break;											\
		msleep(100);											\
	}														\
}

bool Synaptics_PowerOnOff (struct synaptics_ts_data* hTouch, bool OnOff)
{
#if defined (CONFIG_SU880) || defined (CONFIG_KU8800) || defined (CONFIG_LU8800) || defined (CONFIG_KS1103)    
	/* Reset Pin Control */
	gpio_set_value(TEGRA_GPIO_PK6, OnOff);		  
	if(OnOff) mdelay(10);
#endif
	DEBUG_MSG(E, "[TOUCH] Synaptics_PowerOnOff : %d !!\n",OnOff);
	bd6084_bl_set_ldo(BD6084_LDO4, (int)OnOff);
	bd6084_bl_set_ldo(BD6084_LDO3, (int)OnOff);

#if !defined (CONFIG_KS1103) //                             
	if(OnOff)
		mdelay(400); //woo.jung Synaptics Guide for Booting Time in Touch Device
#endif
	
    return true;

err_pmu_open:
    return false;
}

bool powerOn(void* h_dev)
{	
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;

	DO_F(Synaptics_PowerOnOff(hTouch, true), err_powerOn);
	return true;
err_powerOn:
	return false;
}

void powerOff(void* h_dev)
{	
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;

	Synaptics_PowerOnOff(hTouch, false);
}

void init_touch_power(void* h_dev, struct touch_power_func* h_powerCtrl)
{
	h_powerCtrl->on = powerOn;
	h_powerCtrl->off = powerOff;
}

static int touch_status_not_abs0_cnt = 0;  //                                       

bool Synaptics_GetData (void* h_dev, struct touch_finger_data* data)
{
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;
	u16 touch_finger_bit_mask=0;
	u8  finger_index=0;
	u8  index=0;

	//                                                                  
	if(i2c_smbus_read_i2c_block_data(hTouch->client, SYNAPTICS_INT_STATUS_REG, sizeof(u8)*4, (u8*)(&ts_reg_data.interrupt_status_reg)) < 0)
	{
		DEBUG_MSG(E,"[TOUCH] ERROR I2C READ FAIL SYNAPTICS_INT_STATUS_REG\n");
		goto touch_reset;
	}
	//                                                                 
	DEBUG_MSG(M, "[TOUCH] i[%x], 0[%x], 1[%x], 2[%x]\n", ts_reg_data.interrupt_status_reg, ts_reg_data.finger_state_reg[0], ts_reg_data.finger_state_reg[1], ts_reg_data.finger_state_reg[2]);

	//                                                  
	if(i2c_smbus_read_i2c_block_data(hTouch->client, SYNAPTICS_DATA_BASE_REG, sizeof(u8), (u8*)(&ts_reg_data.device_status_reg)) < 0 )
	{
		DEBUG_MSG(E,"[TOUCH] ERROR I2C READ FAIL SYNAPTICS_DATA_BASE_REG\n");
		goto touch_reset;
	}

#if 1
	//                                                                                               

	//                                                                           
	/* // Temp Code
	if ( ((int)ts_reg_data.device_status_reg & 0x03) == 0x03 || ((int)ts_reg_data.device_status_reg & 0x80) == 0x80 ) //Touch Reset Doing for ESD or Drop Test
	{
		if ( ((int)ts_reg_data.device_status_reg & 0x03) == 0x03)
			printk("Synaptics_GetData HW RESET - ts_reg_data.device_status_reg & 0x03!");
				
		else if (((int)ts_reg_data.device_status_reg & 0x80) == 0x80)
			printk("Synaptics_GetData HW RESET - ts_reg_data.device_status_reg & 0x80!");
								
		goto touch_reset;
	}
	*/

    if ( ((int)ts_reg_data.device_status_reg & 0x03) == 0x03) //Touch Reset Doing for ESD or Drop Test
	{
	    	printk("Synaptics_GetData HW RESET - ts_reg_data.device_status_reg & 0x03!\n");
  	    goto touch_reset;
   	}
	//                                                                                               
#else
	if (synaptics_ts_hw_reset(hTouch, ts_reg_data.device_status_reg) == true)
		return false;
#endif
	//                                                  
	if(ts_reg_data.interrupt_status_reg == SYNAPTICS_INT_ABS0){
		touch_status_not_abs0_cnt = 0;  //                                       
		touch_finger_bit_mask = GET_BIT_MASK(ts_reg_data.finger_state_reg);
#if defined(CONFIG_SU880) || defined(CONFIG_KU8800) || defined(CONFIG_KS1103) //                                        
		data->touch_finger_bit_mask = touch_finger_bit_mask;
#endif
		while(touch_finger_bit_mask){
			GET_INDEX_FROM_MASK(finger_index, touch_finger_bit_mask)
			if(i2c_smbus_read_i2c_block_data(hTouch->client, SYNAPTICS_REG_FINGER_DATA_START_ADDR + (SYNAPTICS_REG_FINGER_DATA_GAP*finger_index), 
										       SYNAPTICS_REG_FINGER_VALID_DATA_SIZE, ts_reg_data.fingers_data[index]) < 0)
			{
				DEBUG_MSG(E,"[TOUCH] ERROR I2C READ FAIL SYNAPTICS_REG_FINGER_DATA\n");
				goto touch_reset;
			}
#if defined(CONFIG_SU880) || defined(CONFIG_KU8800) || defined(CONFIG_KS1103) //                                        
			if((data->prev_touch_finger_bit_mask == 1) && (data->touch_finger_bit_mask == 2))
			{
				index++;
			}
#endif
			data->curr_data[index].X_position = TS_SNTS_GET_X_POSITION(ts_reg_data.fingers_data[index][X_HIGH_POSITION], ts_reg_data.fingers_data[index][XY_LOW_POSITION]);
			data->curr_data[index].Y_position = TS_SNTS_GET_Y_POSITION(ts_reg_data.fingers_data[index][Y_HIGH_POSITION], ts_reg_data.fingers_data[index][XY_LOW_POSITION]); 	
			data->curr_data[index].width		= TS_SNTS_GET_WIDTH(ts_reg_data.fingers_data[index][XY_WIDTH]);
			data->curr_data[index].pressure	= TS_SNTS_GET_PRESSURE(ts_reg_data.fingers_data[index][PRESSURE]);
#if defined(CONFIG_SU880) || defined(CONFIG_KU8800) || defined(CONFIG_KS1103) //                                        
			data->curr_data[index].id = finger_index;
#endif
			DEBUG_MSG(M, "[TOUCH] X[%d], Y[%d], Press[%d], Width[%d]", (int)data->curr_data[index].X_position, (int)data->curr_data[index].Y_position, (int)data->curr_data[index].width, (int)data->curr_data[index].pressure);
		
			tegra_touch_adjust_position(data->prev_data[index].X_position, data->prev_data[index].Y_position, &data->curr_data[index].X_position, &data->curr_data[index].Y_position);
#if defined (LGE_NOMELT)
			current_data_x = TS_SNTS_GET_LOWDATA_X_POSITION(ts_reg_data.fingers_data[index][X_HIGH_POSITION], ts_reg_data.fingers_data[index][XY_LOW_POSITION]);
			current_data_y = TS_SNTS_GET_LOWDATA_Y_POSITION(ts_reg_data.fingers_data[index][Y_HIGH_POSITION], ts_reg_data.fingers_data[index][XY_LOW_POSITION]);
			DEBUG_MSG(B,"[TOUCH] current_data_x[%d], current_data_y[%d]\n",current_data_x, current_data_y);
#endif /*            */
			index++;
		}
		data->total_num = index;
		
#if defined (LGE_NOMELT)
		Synaptics_SetNoMeltMode (h_dev, false);
#endif /*            */
		return true;
	}
	//                                                                  
	else if (ts_reg_data.interrupt_status_reg == SYNAPTICS_INT_FLASH){
			DEBUG_MSG(E,"[TOUCH] TOUCH_IC = Download Mode => Touch RESET!!\n");
			goto touch_reset;
		return false;
	}
	//                                                                 
	else {
	//                                           
		touch_status_not_abs0_cnt ++;
		if(touch_status_not_abs0_cnt == 5)
		{
			DEBUG_MSG(E,"[TOUCH] touch_status_not_abs0_cnt Touch RESET!! go !!\n");
			touch_status_not_abs0_cnt = 0;
			goto touch_reset;
		}
		return false;
	}
	//                                           
touch_reset:
	if(touch_reset_flag != true)
	{
		DEBUG_MSG(E,"[TOUCH] Touch_Reset_Flag is False so Task can reset!");
		touch_status_not_abs0_cnt = 0;  //                                       
		synaptics_ts_hw_reset(hTouch);
	} 
	else { 
		DEBUG_MSG(E,"[TOUCH] Skip Touch Reset When the other task is resetting for ESD!!!");		
	}
	return false;
}

u8 Synaptics_CheckButton(void* h_dev, struct touch_finger_data* data)
{
	u8 tmp_button = KEY_NULL;
	u8 sync = DO_NOT_ANYTHING;
	u8 total_num = data->total_num;
	u8 prev_button = data->prev_button;
	u8 state = data->state;
	static u8 touch_lock_cnt = 0;
	struct touch_data curr_data = data->curr_data[0];

	if(total_num == FINGER_RELEASE) goto err_its_release;
	if(touch_lock_cnt > 20) state = DO_NOT_ANYTHING;
	if(state == TOUCH_LOCK) goto err_its_lock;
	touch_lock_cnt = 0;
	if(total_num != SINGLE_FINGER) goto err_its_multi;

#if defined (CONFIG_SU880) || defined (CONFIG_KU8800) || defined (CONFIG_LU8800) || defined (CONFIG_KS1103)  
	if(IS_PANEL(curr_data.Y_position))			tmp_button = KEY_PANEL;
	else if(IS_MENU(curr_data.X_position))		tmp_button = KEY_MENU;
	else if(IS_HOME(curr_data.X_position))		tmp_button = KEY_HOME;
	else if(IS_BACK(curr_data.X_position))		tmp_button = KEY_BACK;
	else if(IS_SEARCH(curr_data.X_position))	tmp_button = KEY_SEARCH;
	else										tmp_button = KEY_BOUNDARY;

#else
	if(IS_PANEL(curr_data.Y_position))			tmp_button = KEY_PANEL;
	else if(IS_MENU(curr_data.X_position))		tmp_button = KEY_MENU;
	else if(IS_HOME(curr_data.X_position))		tmp_button = KEY_HOME;
	else if(IS_BACK(curr_data.X_position))		tmp_button = KEY_BACK;
	else if(IS_SEARCH(curr_data.X_position))	tmp_button = KEY_SEARCH;
	else										tmp_button = KEY_BOUNDARY;
#endif

	if(prev_button != KEY_NULL && prev_button != KEY_BOUNDARY){
		if(prev_button == KEY_PANEL){
			if(prev_button != tmp_button) sync = ABS_RELEASE;
			else sync = ABS_PRESS;
		}
		else{
			if(prev_button != tmp_button) sync = BUTTON_RELEASE;
			else sync = DO_NOT_ANYTHING;
		}
	}
	else{
		if(tmp_button == KEY_PANEL) sync = ABS_PRESS;
		else if(tmp_button == KEY_BOUNDARY) sync = DO_NOT_ANYTHING;
		else sync = BUTTON_PRESS;
	}

	data->curr_button = tmp_button;
	return sync;

err_its_multi:
	data->curr_button = KEY_PANEL;
	if(prev_button && prev_button != KEY_PANEL)	return BUTTON_RELEASE;
	else return ABS_PRESS;

err_its_lock:
	touch_lock_cnt++;
	return TOUCH_LOCK;
	
err_its_release:
	data->curr_button = KEY_NULL;
	if(prev_button){
		if(prev_button == KEY_PANEL) return ABS_RELEASE;
		else return BUTTON_RELEASE;
	}
	else
		return DO_NOT_ANYTHING;	
}

bool Synaptics_AddJob (void* h_dev, struct touch_finger_data* data, u32 whereis)
{
	switch(whereis){
		case BEFORE_WHILE:
			set_freezable_with_signal();
			break;
		case AFTER_GET_DATA:
			break;
		case AFTER_SYNC:
#if 0  //                                        
			if(data->total_num == SINGLE_FINGER){
				if(data->state == ABS_RELEASE)
					data->state = TOUCH_LOCK;
			}
#endif
			break;
		default:
			break;
	}
	return true;
}

void init_touch_finger_data(void* h_dev, struct touch_finger_data_func* h_fingerData)
{
	h_fingerData->get_finger_data = Synaptics_GetData;
	h_fingerData->check_button = Synaptics_CheckButton;
	h_fingerData->additional_job = Synaptics_AddJob;
}

void init_touch_device_caps(void* h_dev, struct touch_device_caps* h_caps)
{
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;
    h_caps = &hTouch->caps;
}

bool init_touch_device_setting(void* h_dev, bool fw_update)
{
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;

#ifdef STAR_FW_UPGRADE
	if (fw_update == true)
		DO_C(synaptics_ts_fw_upgrade(hTouch) != 0, err_Specific_Device_Setting);
#endif

	i2c_smbus_write_byte_data(hTouch->client, SYNAPTICS_INTERRUPT_ENABLE_REG, SYNAPTICS_INT_ABS0);

	i2c_smbus_write_byte_data(hTouch->client, SYNAPTICS_DELTA_X_THRES_REG, SYNAPTICS_DELTA_THRESHOLD);	// Delta X
	i2c_smbus_write_byte_data(hTouch->client, SYNAPTICS_DELTA_Y_THRES_REG, SYNAPTICS_DELTA_THRESHOLD);	// Delta Y

	i2c_smbus_write_byte_data(hTouch->client, SYNAPTICS_2D_GESTURE_ENABLE1, 0x00);		// 2d gesture enable1 = not use
	i2c_smbus_write_byte_data(hTouch->client, SYNAPTICS_2D_GESTURE_ENABLE2, 0x00);		// 2d gesture enable2 = not use

	i2c_smbus_write_byte_data(hTouch->client, SYNAPTICS_REPORT_MODE_REG, 0x08);		// continuous reporting

#if defined (LGE_NOMELT)
	Synaptics_SetNoMeltMode ((void*)0, true);
#else
	i2c_smbus_write_byte_data(hTouch->client, SYNAPTICS_MELT_CONTROL, SYNAPTICS_NO_MELT);
#endif /*            */

	i2c_smbus_read_i2c_block_data(hTouch->client, SYNAPTICS_INT_STATUS_REG, sizeof(u8)*4, (u8*)(&ts_reg_data.interrupt_status_reg));

	//                                                                 
//	if(gpio_get_value(TEGRA_GPIO_PX6) == 0) // Interrupt Pin Check is HIGH
//		printk("IRQ PIN is Not HIGH!!! Can't Operate Touch!!\n");
	//                                                                 

	
	return true;
err_Specific_Device_Setting:
	return false;
}

//                                                  
bool touch_device_sleep_mode_setting(void* h_dev, bool mode)
{
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;

	if( mode == SLEEP_IN )
	{
		i2c_smbus_write_byte_data(hTouch->client, SYNAPTICS_DEVICE_CONTROL_REG, 0x01);
	}
	else if( mode == SLEEP_OUT)
	{
		i2c_smbus_write_byte_data(hTouch->client, SYNAPTICS_DEVICE_CONTROL_REG, 0x02);
		i2c_smbus_write_byte_data(hTouch->client, SYNAPTICS_MELT_CONTROL, SYNAPTICS_MELT);
	}
	return true;
err_Sleep_Mode_Setting:
	return false;
}
//                                                  

#if defined (LGE_NOMELT)
/*
                          
                           
                                                                 
                                                                                              
                                       
 */
static void Synaptics_SetNoMeltMode (void* h_dev, bool binit)
{
	struct synaptics_ts_data* hTouch;
	hTouch = (struct synaptics_ts_data*)h_dev;

    if (binit)
    {
        mode = 1;
        numfinger = 0;
        reportcnt = 0;
		tapcount = 0;
    }
    else 
	if (mode)
	{
		if((ts_reg_data.finger_state_reg[0] == 0) & (ts_reg_data.finger_state_reg[1] == 0) & (ts_reg_data.finger_state_reg[2] == 0)) //No finger
		{
			DEBUG_MSG(E, "[TOUCH] numfinger=%d,reportcnt=%d\n",numfinger,reportcnt);
			if((numfinger==1) & (reportcnt > 6)) 
			{
				DEBUG_MSG(E, "[TOUCH] firstx=%d,firsty=%d\n",firstx,firsty);
				if((abs(firstx - current_data_x) > 200) | (abs(firsty - current_data_y) >200)) //correspond to 1cm
				{
					if(mode==1)
					{
						DEBUG_MSG(E, "[TOUCH] Mode = 1, firstx=%d,firsty=%d\n",firstx,firsty);					
						if(i2c_smbus_write_byte_data(hTouch->client, SYNAPTICS_MELT_CONTROL,SYNAPTICS_NO_MELT)<0) //set no melting
						{
							DEBUG_MSG(E,"[TOUCH] ERROR I2C WRITE FAIL SYNAPTICS_MELT_CONTROL\n");
						}
						mode = 0;
						DEBUG_MSG(E, "[TOUCH] No melt mode~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
						tapcount=0;
					}
					else if(mode==2)
					{
						if(tapcount>2)
						{
							DEBUG_MSG(E, "[TOUCH] Mode = 2, firstx=%d,firsty=%d\n",firstx,firsty);											
							if(i2c_smbus_write_byte_data(hTouch->client, SYNAPTICS_MELT_CONTROL,SYNAPTICS_NO_MELT)<0) //set no melting
							{
								DEBUG_MSG(E,"[TOUCH] ERROR I2C WRITE FAIL SYNAPTICS_MELT_CONTROL\n");
							}
							mode = 0;
							DEBUG_MSG(E, "[TOUCH] No melt mode~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
							tapcount=0;
						}
					}
				}
			}

			if(numfinger==0) 
			{
				mode=2;
				tapcount=0;
			}

			numfinger=0;
			reportcnt=0;
		}

		else if((ts_reg_data.finger_state_reg[0] == 1) & (ts_reg_data.finger_state_reg[1] == 0) & (ts_reg_data.finger_state_reg[2] == 0)) // 1 finger
		{
			if(++reportcnt > 10) reportcnt=10;
			if(numfinger==0)
			{
				numfinger=1;
				firstx=current_data_x; firsty=current_data_y;
				prex=current_data_x; prey=current_data_y;
				tapcount++;
			}
			else if(numfinger==1)
			{
				if((abs(prex-current_data_x) > 500) | (abs(prey-current_data_y) > 500)) 
				{
					numfinger=2;
					tapcount=0;
					mode=2;
				}
				prex=current_data_x; prey=current_data_y;
			}
		}
		else
		{
			numfinger=2; // more than 2 finger
			tapcount=0;
			mode=2;
		}
	}
}
#endif /*            */


#ifdef STAR_FW_UPGRADE

#define TOUCH_FW_COMPARE

#define WAIT_UNTIL_PIN_READY(_pin)	\
{	\
	while(gpio_get_value(_pin)){	\
		mdelay(1);	\
	}	\
}

#define WAIT_UNTIL_FLASH_CMD_READY(_cond)\
{	\
	u8 flashValue, temp_data;	\
	do{	\
		flashValue = i2c_smbus_read_byte_data(client, SYNAPTICS_FLASH_CONTROL_REG);	\
		temp_data = i2c_smbus_read_byte_data(client, SYNAPTICS_INT_STATUS_REG);	\
	} while(_cond);\
}

#define WAIT_UNTIL_DEVICE_READY(_cond, _pin)\
		WAIT_UNTIL_PIN_READY(_pin)	\
		WAIT_UNTIL_FLASH_CMD_READY(_cond)

#define I2C_R(_state)	DO_SAFE(_state, "I2C READ", return -1)
#define I2C_W(_state)	DO_SAFE(_state, "I2C WRITE", return -1)


static u8 Synaptics_GetFWVersion(struct i2c_client *client)
{
	u8 RMI_Query_BaseAddr;
	u8 FWVersion_Addr;

	u8 SynapticsFirmVersion;

	I2C_R((RMI_Query_BaseAddr = i2c_smbus_read_byte_data(client, SYNAPTICS_RMI_QUERY_BASE_REG)) < 0)
	FWVersion_Addr = RMI_Query_BaseAddr+3;
	I2C_R((SynapticsFirmVersion = i2c_smbus_read_byte_data(client, FWVersion_Addr)) < 0)

	DEBUG_MSG(3, "[TOUCH FW] synaptics_GetFWVersion = %x\n", SynapticsFirmVersion)

	return SynapticsFirmVersion;
}

static unsigned long ExtractLongFromHeader(const u8 *SynaImage)  // Endian agnostic 
{
  return((unsigned long)SynaImage[0] +
         (unsigned long)SynaImage[1]*0x100 +
         (unsigned long)SynaImage[2]*0x10000 +
         (unsigned long)SynaImage[3]*0x1000000);
}

static void CalculateChecksum(u16 *data, u16 len, u32 *dataBlock)
{
  unsigned long temp = *data++;
  unsigned long sum1;
  unsigned long sum2;

  *dataBlock = 0xffffffff;

  sum1 = *dataBlock & 0xFFFF;
  sum2 = *dataBlock >> 16;

  while (len--)
  {
    sum1 += temp;    
    sum2 += sum1;    
    sum1 = (sum1 & 0xffff) + (sum1 >> 16);    
    sum2 = (sum2 & 0xffff) + (sum2 >> 16);
  }

  *dataBlock = sum2 << 16 | sum1;
}

static void SpecialCopyEndianAgnostic(u8 *dest, u16 src) 
{
  dest[0] = src%0x100;  //Endian agnostic method
  dest[1] = src/0x100;  
}

#ifdef TOUCH_FW_COMPARE
//                                                    
#if defined (SYNAPTICS_TOUCH_FW_UPGRADE)
static int fw_compare(struct synaptics_fw_data *tsfw, struct i2c_client *client, const u8 BlockDataStartAddr, u16 index, const u16 block_size)
#else
static int fw_compare(struct i2c_client *client, const u8 BlockDataStartAddr, u16 index, const u16 block_size)
#endif
//                                                    
{
	u8  *tmp_block = kmalloc(sizeof(u8)*block_size, GFP_KERNEL);
	u8	i;
	
	if(i2c_smbus_read_i2c_block_data(client, BlockDataStartAddr, sizeof(tmp_block), tmp_block) < sizeof(tmp_block)){
		kfree(tmp_block);
		return -1;
	}
	
	for(i=0; i<sizeof(tmp_block); i++){
//                                                    
#if defined (SYNAPTICS_TOUCH_FW_UPGRADE)
		if(unlikely(tmp_block[i] != tsfw->image_bin[index])){

#else
		if(unlikely(tmp_block[i] != SynapticsFirmware[index])){
#endif
//                                                    
			kfree(tmp_block);
			return -1;
		}
//                                                    
#if defined (SYNAPTICS_TOUCH_FW_UPGRADE)
		DEBUG_MSG(1, "[TOUCH FW] [%x] : tmp[%x] / Firm[%x]\n", i, tmp_block[i], tsfw->image_bin[index]);
#else
		DEBUG_MSG(1, "[TOUCH FW] [%x] : tmp[%x] / Firm[%x]\n", i, tmp_block[i], SynapticsFirmware[index]);
#endif
//                                                    

		index++;
	}

	//mdelay(100);
	kfree(tmp_block);
	return 0;
}
#endif

static int synaptics_ts_fw_upgrade(struct synaptics_ts_data* hTouch)
{
	struct i2c_client *client = hTouch->client;
//                                                    
#if defined (SYNAPTICS_TOUCH_FW_UPGRADE)
	struct synaptics_fw_data *tsfw;
	int fd = -1;   
	struct stat fw_bin_stat;	
	mm_segment_t old_fs = 0;
	unsigned long read_bytes;
#endif
//                                                    

	int i;
	
	u8 TouchFWVersion;

	u8 FlashQueryBaseAddr, FlashDataBaseAddr;
	u8 RMICommandBaseAddr;
	
	u8 BootloaderIDAddr;
	u8 BlockSizeAddr;
	u8 FirmwareBlockCountAddr;
	u8 ConfigBlockCountAddr;

	u8 BlockNumAddr;
	u8 BlockDataStartAddr;
	
	u8 bootloader_id[2];

	u8 temp_array[2], temp_data, m_firmwareImgVersion;
	u8 checkSumCode;

	u16 ts_block_size, ts_config_block_count, ts_fw_block_count;
	u16 m_bootloadImgID;
	
	u32 ts_config_img_size;
	u32 ts_fw_img_size;
	u32 m_fileSize, m_firmwareImgSize, m_configImgSize, m_FirmwareImgFile_checkSum;

	u8 RMI_Query_BaseAddr;
	u8 product_id_addr;

	u8 product_id[7];

	////////////////////////////////////////////////////////////////////////////////////

	DEBUG_MSG(3, "[TOUCH FW] Synaptics_UpgradeFirmware :: BQ: TM1765, X2 : TM1962 [START]\n")

//                                                   
//#if defined (CONFIG_SU880) || defined (CONFIG_KU8800) || defined (CONFIG_LU8800) || defined (CONFIG_KS1103)
#if !defined (CONFIG_KS1103) //                             
    msleep(500); // temp
#endif
//#endif
//                                                   

//                                                    
#if defined (SYNAPTICS_TOUCH_FW_UPGRADE)
	tsfw = kzalloc(sizeof(*tsfw), GFP_KERNEL);
	DEBUG_MSG(E,"[TOUCH FW]tsfw memory = 0x%x\n",tsfw);  // sghyun_msg
	if (tsfw == NULL) {
		DEBUG_MSG(E,"Can not allocate  memory\n");
		kfree(tsfw);
		return -1;
	}

	if(unlikely(hTouch->fw_path[0] != 0)) {
		old_fs = get_fs();
		set_fs(get_ds());

		if ((fd = sys_open((const char __user *) hTouch->fw_path, O_RDONLY, 0)) < 0) {
			DEBUG_MSG(E,"Can not read FW binary from %s\n", hTouch->fw_path);
			set_fs(old_fs);
			kfree(tsfw);
			return -1;
		}

		if ((sys_newstat(hTouch->fw_path, (struct stat *)&fw_bin_stat)) < 0) {
			DEBUG_MSG(E,"Can not read FW binary stat from %s\n", hTouch->fw_path);
			sys_close(fd);
			set_fs(old_fs);
			kfree(tsfw);
			return -1;
		}

		tsfw->image_size = fw_bin_stat.st_size;
		tsfw->image_bin = kzalloc(sizeof(char) * (tsfw->image_size+1), GFP_KERNEL);
		if (tsfw->image_bin == NULL) {
			DEBUG_MSG(E,"Can not allocate  memory\n");
			sys_close(fd);
			set_fs(old_fs);
			kfree(tsfw);
			return -1;
		}

		//sys_lseek(fd, (off_t) pos, 0);
		read_bytes = sys_read(fd, (char __user *)tsfw->image_bin, tsfw->image_size);

		/* for checksum */
		*(tsfw->image_bin+tsfw->image_size) = 0xFF;

		DEBUG_MSG(E,"Touch FW image read %ld bytes from %s\n", read_bytes, hTouch->fw_path);
	}else {
		tsfw->image_size = sizeof(SynapticsFirmware)-1;
		tsfw->image_bin = (unsigned char *)(&SynapticsFirmware[0]);
	}
#endif
//                                                    

	////////////////////////	Product ID Check	///////////////////////////
	
	
	I2C_R((RMI_Query_BaseAddr = i2c_smbus_read_byte_data(client, SYNAPTICS_RMI_QUERY_BASE_REG)) < 0)
	product_id_addr = RMI_Query_BaseAddr+11;

	I2C_R(i2c_smbus_read_i2c_block_data(client, product_id_addr, sizeof(product_id)-1, product_id) < sizeof(product_id)-1)
	product_id[6] = '\0';

	DEBUG_MSG(E, "[TOUCH FW] Touch controller Product ID = %s\n", product_id)
//                                                    
#if defined (SYNAPTICS_TOUCH_FW_UPGRADE)
	DO_SAFE(strncmp(product_id, &tsfw->image_bin[0x10], 6) != 0, "", return 0)
#else
	DO_SAFE(strncmp(product_id, &SynapticsFirmware[0x10], 6) != 0, "", return 0)
#endif
//                                                    

	DO_SAFE((TouchFWVersion = Synaptics_GetFWVersion(client)) == -1, "", return 0)
//                                                    
#if defined (SYNAPTICS_TOUCH_FW_VERSION)
	hTouch->fw_rev = TouchFWVersion;
#endif
//                                                    

//                                                    
#if defined (SYNAPTICS_TOUCH_FW_UPGRADE)
	if((TouchFWVersion >= 0x64 && tsfw->image_bin[0x1F] >= 0x64) || (TouchFWVersion < 0x64 && tsfw->image_bin[0x1F] < 0x64))
	{
		DO_SAFE(!(TouchFWVersion < tsfw->image_bin[0x1F]), "FW Upgrade is not needed", return 0)
	}
#else
	if((TouchFWVersion >= 0x64 && SynapticsFirmware[0x1F] >= 0x64) || (TouchFWVersion < 0x64 && SynapticsFirmware[0x1F] < 0x64))
	{
		DO_SAFE(!(TouchFWVersion < SynapticsFirmware[0x1F]), "FW Upgrade is not needed", return 0)
	}
#endif
//                                                    

	////////////////////////	Configuration	///////////////////////////
	I2C_R((FlashQueryBaseAddr =  i2c_smbus_read_byte_data(client, SYNAPTICS_FLASH_QUERY_BASE_REG)) < 0)

	BootloaderIDAddr = FlashQueryBaseAddr;
	BlockSizeAddr = FlashQueryBaseAddr + 3;
	FirmwareBlockCountAddr = FlashQueryBaseAddr + 5;
	ConfigBlockCountAddr = FlashQueryBaseAddr + 7;
	
	I2C_R((FlashDataBaseAddr = i2c_smbus_read_byte_data(client, SYNAPTICS_FLASH_DATA_BASE_REG)) < 0)

	BlockNumAddr = FlashDataBaseAddr;
	BlockDataStartAddr = FlashDataBaseAddr + 2;

//                                                    
#if defined (SYNAPTICS_TOUCH_FW_UPGRADE)
	m_fileSize = tsfw->image_size;

	checkSumCode         = ExtractLongFromHeader(&(tsfw->image_bin[0]));
	m_bootloadImgID      = (unsigned int)tsfw->image_bin[4] + (unsigned int)tsfw->image_bin[5]*0x100;
	m_firmwareImgVersion = tsfw->image_bin[7]; 
	m_firmwareImgSize    = ExtractLongFromHeader(&(tsfw->image_bin[8]));
	m_configImgSize      = ExtractLongFromHeader(&(tsfw->image_bin[12]));    

	CalculateChecksum((u16*)&(tsfw->image_bin[4]), (u16)(m_fileSize-4)>>1, &m_FirmwareImgFile_checkSum);
#else
	m_fileSize = sizeof(SynapticsFirmware) -1;

	checkSumCode         = ExtractLongFromHeader(&(SynapticsFirmware[0]));
	m_bootloadImgID      = (unsigned int)SynapticsFirmware[4] + (unsigned int)SynapticsFirmware[5]*0x100;
	m_firmwareImgVersion = SynapticsFirmware[7]; 
	m_firmwareImgSize    = ExtractLongFromHeader(&(SynapticsFirmware[8]));
	m_configImgSize      = ExtractLongFromHeader(&(SynapticsFirmware[12]));    

	CalculateChecksum((u16*)&(SynapticsFirmware[4]), (u16)(m_fileSize-4)>>1, &m_FirmwareImgFile_checkSum);
#endif
//                                                    

	// Get Current Firmware Information
	I2C_R(i2c_smbus_read_i2c_block_data(client, BlockSizeAddr, sizeof(temp_array), (u8*)&temp_array[0]) < sizeof(temp_array)) 
	ts_block_size = temp_array[0] + (temp_array[1] << 8);

	I2C_R(i2c_smbus_read_i2c_block_data(client,FirmwareBlockCountAddr, sizeof(temp_array), (u8*)&temp_array[0]) < sizeof(temp_array))
	ts_fw_block_count = temp_array[0] + (temp_array[1] << 8);
	ts_fw_img_size = ts_block_size * ts_fw_block_count;

	I2C_R(i2c_smbus_read_i2c_block_data(client, ConfigBlockCountAddr, sizeof(temp_array), (u8*)&temp_array[0]) < sizeof(temp_array)) 
	ts_config_block_count = temp_array[0] + (temp_array[1] << 8);
	ts_config_img_size = ts_block_size * ts_config_block_count;

	I2C_R(i2c_smbus_read_i2c_block_data(client, BootloaderIDAddr, sizeof(bootloader_id), (u8*)&bootloader_id[0]) < sizeof(bootloader_id))

	// Compare
	DO_SAFE(m_fileSize != (0x100+m_firmwareImgSize+m_configImgSize), "", return 0)
	DO_SAFE(m_firmwareImgSize != ts_fw_img_size, "", return 0)
	DO_SAFE(m_configImgSize != ts_config_img_size, "", return 0)
	DO_SAFE(m_firmwareImgVersion == 0 && ((unsigned int)bootloader_id[0] + (unsigned int)bootloader_id[1]*0x100) != m_bootloadImgID, "", return 0)

	////////////////////////	Flash Command - Enable	///////////////////////////
	I2C_W(i2c_smbus_write_i2c_block_data(client, BlockDataStartAddr, sizeof(bootloader_id), bootloader_id) < 0)
	WAIT_UNTIL_FLASH_CMD_READY((flashValue & 0x0f) != 0x00)

	I2C_W(i2c_smbus_write_byte_data(client, SYNAPTICS_FLASH_CONTROL_REG, SYNAPTICS_FLASH_CMD_ENABLE) < 0)
	WAIT_UNTIL_DEVICE_READY(flashValue != 0x80, hTouch->gpio)

	DEBUG_MSG(E, "[TOUCH FW] Synaptics_UpgradeFirmware :: Flash Program Enable Setup Complete\n")

	////////////////////////	Flash Command  - Eraseall	///////////////////////////
	I2C_W(i2c_smbus_write_i2c_block_data(client, BlockDataStartAddr, sizeof(bootloader_id), bootloader_id) < 0)
	I2C_W(i2c_smbus_write_byte_data(client, SYNAPTICS_FLASH_CONTROL_REG, SYNAPTICS_FLASH_CMD_ERASEALL) < 0)
	
	WAIT_UNTIL_DEVICE_READY(flashValue != 0x80, hTouch->gpio)
	
	DEBUG_MSG(E, "[TOUCH FW] Synaptics_UpgradeFirmware :: Flash Erase Complete\n")

	////////////////////////	F/W Data Write	///////////////////////////
	for(i = 0; i < ts_fw_block_count; ++i)
	{
		temp_array[0] = i & 0xff;
		temp_array[1] = (i & 0xff00) >> 8;

		I2C_W(i2c_smbus_write_i2c_block_data(client, BlockNumAddr, sizeof(temp_array), temp_array) < 0)
//                                                    
#if defined (SYNAPTICS_TOUCH_FW_UPGRADE)
		I2C_W(i2c_smbus_write_i2c_block_data(client, BlockDataStartAddr, ts_block_size, (u8*)&tsfw->image_bin[0x100+i*ts_block_size]) < 0)
#else
		I2C_W(i2c_smbus_write_i2c_block_data(client, BlockDataStartAddr, ts_block_size, (u8*)&SynapticsFirmware[0x100+i*ts_block_size]) < 0)
#endif
//                                                    

#ifdef TOUCH_FW_COMPARE
//                                                    
#if defined (SYNAPTICS_TOUCH_FW_UPGRADE)
		if(fw_compare(tsfw, client, BlockDataStartAddr, 0x100+i*ts_block_size, ts_block_size)){
#else
		if(fw_compare(client, BlockDataStartAddr, 0x100+i*ts_block_size, ts_block_size)){
#endif
//                                                    

			DEBUG_MSG(3, "[TOUCH FW] FAIL: Firmware Update[%x]\n", i)
			return -1;
		}
#endif	
		I2C_W(i2c_smbus_write_byte_data(client, SYNAPTICS_FLASH_CONTROL_REG, SYNAPTICS_FLASH_CMD_FW_WRITE) < 0)
		WAIT_UNTIL_DEVICE_READY(flashValue != 0x80, hTouch->gpio)
	}
	DEBUG_MSG(E, "[TOUCH FW] Synaptics_UpgradeFirmware :: Flash Firmware Write Complete\n")

	////////////////////////	F/W Config Write	///////////////////////////
	for(i = 0; i < ts_config_block_count; i++)
	{
		SpecialCopyEndianAgnostic(&temp_array[0], i);

		I2C_W(i2c_smbus_write_i2c_block_data(client, BlockNumAddr, sizeof(temp_array), temp_array) < 0)
//                                                    
#if defined (SYNAPTICS_TOUCH_FW_UPGRADE)
		I2C_W(i2c_smbus_write_i2c_block_data(client, BlockDataStartAddr, ts_block_size, (u8*)&tsfw->image_bin[0x100+m_firmwareImgSize+i*ts_block_size]) < 0)
#else
		I2C_W(i2c_smbus_write_i2c_block_data(client, BlockDataStartAddr, ts_block_size, (u8*)&SynapticsFirmware[0x100+m_firmwareImgSize+i*ts_block_size]) < 0)
#endif
//                                                    

#ifdef TOUCH_FW_COMPARE
//                                                    
#if defined (SYNAPTICS_TOUCH_FW_UPGRADE)
		if(fw_compare(tsfw, client, BlockDataStartAddr, 0x100+m_firmwareImgSize+i*ts_block_size, ts_block_size)){
#else
		if(fw_compare(client, BlockDataStartAddr, 0x100+m_firmwareImgSize+i*ts_block_size, ts_block_size)){
#endif
//                                                    

			DEBUG_MSG(3, "[TOUCH FW] FAIL: Firmware Update[%x]\n", i)
			return -1;
		}
#endif	
		I2C_W(i2c_smbus_write_byte_data(client, SYNAPTICS_FLASH_CONTROL_REG, SYNAPTICS_FLASH_CMD_CONFIG_WRITE) < 0)
		WAIT_UNTIL_DEVICE_READY(flashValue != 0x80, hTouch->gpio)
	}
	DEBUG_MSG(E, "[TOUCH FW] Synaptics_UpgradeFirmware :: Flash Config Write Complete\n")

	////////////////////////	Reset Touch IC	///////////////////////////
	I2C_W(RMICommandBaseAddr = i2c_smbus_read_byte_data(client, SYNAPTICS_RMI_CMD_BASE_REG) < 0)
	
	if(RMICommandBaseAddr){
		I2C_W(i2c_smbus_write_byte_data(client, RMICommandBaseAddr, 0x01) < 0)				
		mdelay(200);

		WAIT_UNTIL_DEVICE_READY((flashValue & 0x0f) != 0x00, hTouch->gpio)

		I2C_R((temp_data = i2c_smbus_read_byte_data(client, SYNAPTICS_INT_STATUS_REG)) < 0)
	
		// Read F01 Status flash prog, ensure the 6th bit is '0'
		while((temp_data = i2c_smbus_read_byte_data(client, SYNAPTICS_DATA_BASE_REG)) != 0);
	}
	else{
		// H/W reset
		if(!Synaptics_PowerOnOff (hTouch, false))
			return -1;
		if(!Synaptics_PowerOnOff (hTouch, true))
			return -1;
	}
	DEBUG_MSG(E, "[TOUCH] Synaptics_UpgradeFirmware :: Complete!!\n")
//                                                    
#if defined (SYNAPTICS_TOUCH_FW_VERSION)
	mdelay(400);
	DO_SAFE((hTouch->fw_rev = Synaptics_GetFWVersion(client)) == -1, "", return 0)
#endif
//                                                    
//                                                    
#if defined (SYNAPTICS_TOUCH_FW_UPGRADE)
	if(unlikely(hTouch->fw_path[0] != 0))
		kfree(tsfw->image_bin);

	sys_close(fd);
	set_fs(old_fs);
	kfree(tsfw);
#endif
//                                                    

	return 0;	
}
#endif

//                                                  
#if 1
static bool synaptics_ts_hw_reset(struct synaptics_ts_data* hTouch)
{
	touch_reset_flag = true;
	DEBUG_MSG(E, "[TOUCH] ESD synaptics_ts_hw_reset Start!! Touch reset_flag = %d \n",touch_reset_flag);
	interruptDisable(hTouch);
	Synaptics_PowerOnOff(hTouch, false);
	mdelay(20);
	Synaptics_PowerOnOff(hTouch, true);
	mdelay(400);
	interruptEnable(hTouch);
	init_touch_device_setting(hTouch, false);
	touch_reset_flag = false;
	DEBUG_MSG(E, "[TOUCH] ESD synaptics_ts_hw_reset End!! Touch reset_flag = %d \n",touch_reset_flag);	
	return true;
}
#else
static bool synaptics_ts_hw_reset(struct synaptics_ts_data* hTouch, u8 device_status_reg)
{
	if (((int)device_status_reg & 0x03) == 0x03) {
		DEBUG_MSG(E, "[TOUCH] ESD synaptics_ts_hw_reset \n")
		Synaptics_PowerOnOff(hTouch, false);
		mdelay(20);
		Synaptics_PowerOnOff(hTouch, true);
		mdelay(400);
		interruptEnable(hTouch);
		init_touch_device_setting(hTouch, false);
		return true;
	}
	return false;	
}
#endif
//                                                  

