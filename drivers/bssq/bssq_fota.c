#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <asm/gpio.h>
#include <asm/system.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h> //                                                

#if defined (CONFIG_KS1001) || defined (CONFIG_KS1103) || defined(CONFIG_LU6500) || defined(CONFIG_SU880) ||defined(CONFIG_KU8800)
#include <mach/gpio-names.h>

#include "bssq_muic_ti.h"

#if defined(CONFIG_SU880) ||defined(CONFIG_KU8800)
#define GPIO_CP_POWER       TEGRA_GPIO_PV1  //CP_POWER
#endif
#define GPIO_CP_RESET  	    TEGRA_GPIO_PV0
#define GPIO_CP_CTRL       TEGRA_GPIO_PO3

#define GPIO_AP_CP_PATH       TEGRA_GPIO_PU4
#else
/*
#include "nvrm_init.h"
#include "nvodm_query.h"
#include "nvodm_services.h"
#include "mach/nvrm_linux.h"
#include "nvodm_query_discovery.h"
#include "star_muic.h"
*/
#endif 

#define STAR_ERS_TEST_PROC_FILE "driver/fota_test"
extern int create_star_fota_test_proc_file(void);
extern void remove_star_fota_test_proc_file(void);
extern void kernel_restart(char *cmd); //                                           
//extern int fota_ebl_download(void);
struct delayed_work	work; //                                           

typedef enum
{
    USIF_UART,
    USIF_IPC,
    USIF_NONE,
} USIF_MODE_TYPE;

//extern void USIF_ctrl(USIF_MODE_TYPE mode);
//extern void DP3T_Switch_ctrl(DP3T_MODE_TYPE mode);
//                                                    
static void fota_reset_worker(struct work_struct *work)
{
	printk("fota_reset_worker+++\n");
	kernel_restart(NULL);
	printk("fota_reset_worker---\n");
}
//                                                    

void ifx_uart_sw_ctrl( void )
{
    printk("%s\n", __func__);
    //USIF_ctrl(USIF_IPC);
    //DP3T_Switch_ctrl(DP3T_NC);
#if defined(CONFIG_SU880) ||defined(CONFIG_KU8800)
      gpio_request(GPIO_AP_CP_PATH, "ifx_uartpath_n");
      tegra_gpio_enable(GPIO_AP_CP_PATH);
      gpio_direction_output(GPIO_AP_CP_PATH, 1);
     
      gpio_set_value(GPIO_AP_CP_PATH, 1);
      
      mdelay(200);
#endif    
}

void ifx_power_low(void)
{
#if 0//defined(CONFIG_LU6500)
    unsigned int pin_val = 0;
    NvOdmServicesGpioHandle h_gpio;	
    NvOdmGpioPinHandle h_gpiopin;

    printk("%s LOW\n", __func__);
    h_gpio = NvOdmGpioOpen();
    h_gpiopin = NvOdmGpioAcquirePinHandle(h_gpio, 'v' - 'a', 1);
    NvOdmGpioConfig( h_gpio, h_gpiopin, NvOdmGpioPinMode_Output);

    NvOdmGpioSetState( h_gpio, h_gpiopin, 0x0);
    NvOdmGpioGetState( h_gpio, h_gpiopin, &pin_val);
    printk("ifx_power_low - [CP POWER]: pin %d\n", pin_val);

    NvOdmGpioReleasePinHandle(h_gpio, h_gpiopin);
    NvOdmGpioClose(h_gpio);
#endif    
}

void ifx_power_high(void)
{
#if 0 //defined(CONFIG_LU6500)
    unsigned int pin_val = 0;    
    NvOdmServicesGpioHandle h_gpio;	
    NvOdmGpioPinHandle h_gpiopin;

    printk("%s HIGH\n", __func__);
    h_gpio = NvOdmGpioOpen();
    h_gpiopin = NvOdmGpioAcquirePinHandle(h_gpio, 'v' - 'a', 1);
    NvOdmGpioConfig( h_gpio, h_gpiopin, NvOdmGpioPinMode_Output);

    NvOdmGpioSetState( h_gpio, h_gpiopin, 0x1);
    NvOdmGpioGetState( h_gpio, h_gpiopin, &pin_val);
    printk("ifx_power_high - [CP POWER]: pin %d\n", pin_val);    

    NvOdmGpioReleasePinHandle(h_gpio, h_gpiopin);
    NvOdmGpioClose(h_gpio);
#endif
}


void ifx_reset_low(void)
{
#if 0 //defined(CONFIG_LU6500)
    unsigned int pin_val = 0;        
    NvOdmServicesGpioHandle h_gpio;	
    NvOdmGpioPinHandle h_gpiopin;

    printk("%s LOW\n", __func__);    
    h_gpio = NvOdmGpioOpen();
    h_gpiopin = NvOdmGpioAcquirePinHandle(h_gpio, 'v' - 'a', 0);
    NvOdmGpioConfig( h_gpio, h_gpiopin, NvOdmGpioPinMode_Output);

    NvOdmGpioSetState( h_gpio, h_gpiopin, 0x0);
    NvOdmGpioGetState( h_gpio, h_gpiopin, &pin_val);
    printk("ifx_reset_low - [CP RESET]: pin %d\n", pin_val);        

    NvOdmGpioReleasePinHandle(h_gpio, h_gpiopin);
    NvOdmGpioClose(h_gpio);
#endif
}


void ifx_reset_high(void)
{
#if 0 //defined(CONFIG_LU6500)
    unsigned int pin_val = 0;    
    NvOdmServicesGpioHandle h_gpio;	
    NvOdmGpioPinHandle h_gpiopin;

    printk("%s HIGH\n", __func__);    
    h_gpio = NvOdmGpioOpen();
    h_gpiopin = NvOdmGpioAcquirePinHandle(h_gpio, 'v' - 'a', 0);
    NvOdmGpioConfig( h_gpio, h_gpiopin, NvOdmGpioPinMode_Output);

    NvOdmGpioSetState( h_gpio, h_gpiopin, 0x1);
    NvOdmGpioGetState( h_gpio, h_gpiopin, &pin_val);
    printk("ifx_reset_high - [CP RESET]: pin %d\n", pin_val);         

    NvOdmGpioReleasePinHandle(h_gpio, h_gpiopin);
    NvOdmGpioClose(h_gpio);
#endif
}

void ifx_fota_reset(void)
{
    //fota_ebl_download();
#if defined(CONFIG_SU880) ||defined(CONFIG_KU8800)
    int status;
    printk("%s: CP Reset IN\n", __func__);
    gpio_request(GPIO_CP_POWER, "ifx pwron");
    gpio_direction_output(GPIO_CP_POWER, 0);
    udelay(100);
    
//                                                         
#if 1
    gpio_direction_output(GPIO_CP_RESET, 0);  //GPIO_3 OE
    udelay(100);
    gpio_set_value(GPIO_CP_POWER, 1);
    status = gpio_get_value(GPIO_CP_POWER);
    printk("%s: GPIO_CP_POWER low- [CP POWER]: pin %d\n", __func__, status);
    gpio_set_value(GPIO_CP_RESET, 1);
    status = gpio_get_value(GPIO_CP_RESET);
    printk("%s: GPIO_CP_RESET low- [CP RESET]: pin %d\n", __func__, status);
    mdelay(100); // 100mS delay
#endif
//                                                         

    gpio_set_value(GPIO_CP_POWER, 0);
    status = gpio_get_value(GPIO_CP_POWER);
    printk("%s: GPIO_CP_POWER low- [CP POWER]: pin %d\n", __func__, status);
    mdelay(500); // 500mS delay
    
//                                                         
#if 0
    gpio_direction_output(GPIO_CP_RESET, 0);  //GPIO_3 OE
    udelay(100);
#endif
//                                                         

    gpio_set_value(GPIO_CP_RESET, 0);
    status = gpio_get_value(GPIO_CP_RESET);
    printk("%s: GPIO_CP_RESET low- [CP RESET]: pin %d\n", __func__, status);
    mdelay(500); // 500mS delay
    gpio_set_value(GPIO_CP_POWER, 1);
    status = gpio_get_value(GPIO_CP_POWER);
    printk("%s: GPIO_CP_POWER high+ [CP POWER]: pin %d\n", __func__, status);
    mdelay(100); // 1000mS delay
    gpio_set_value(GPIO_CP_RESET, 1);
    status = gpio_get_value(GPIO_CP_RESET);
    printk("%s: GPIO_CP_RESET high+ [CP RESET]: pin %d\n", __func__, status);
//  usif_switch_ctrl(0);
    printk("%s: CP Reset OUT\n", __func__);
#endif
}

int mdm_reset(void)
{
   printk("[LGE_FOTA]%s\n", __func__);

   gpio_request(GPIO_CP_RESET, "mdm_reset_n");
   tegra_gpio_enable(GPIO_CP_RESET);
   gpio_direction_output(GPIO_CP_RESET, 1);
   gpio_set_value(GPIO_CP_RESET, 1);   
   mdelay(400);
   gpio_set_value(GPIO_CP_RESET, 0);
}

#if defined(CONFIG_KS1001) || defined(CONFIG_KS1103) || defined(CONFIG_LU6500)
void CTRL_GPIO_high(void)
{
  printk("[LGE_FOTA]%s\n", __func__);

  gpio_request(GPIO_CP_CTRL, "CTRL_GPIO_high");
  tegra_gpio_enable(GPIO_CP_CTRL);
  gpio_direction_output(GPIO_CP_CTRL, 1);

  gpio_set_value(GPIO_CP_CTRL, 1);

  msleep(3000);
}

void CTRL_GPIO_low(void)
{
  printk("[LGE_FOTA]%s\n", __func__);

  gpio_request(GPIO_CP_CTRL, "CTRL_GPIO_low");
  tegra_gpio_enable(GPIO_CP_CTRL);
  gpio_direction_output(GPIO_CP_CTRL, 0);

  gpio_set_value(GPIO_CP_CTRL, 0);

  msleep(3000);
}
#endif

static ssize_t fota_test_proc_read(struct file *filp, char *buf, size_t len, loff_t *offset)
{
    
    return 1;
}

static ssize_t fota_test_proc_write(struct file *filp, const char *buf, size_t len, loff_t *off)
{
    char messages[10];
    int reg, val;
    int err;
    char cmd;

    if (len > 10)
        len = 10;

    if (copy_from_user(messages, buf, len))
        return -EFAULT;

    sscanf(buf, "%c", &cmd);

    printk("[LGE_FOTA]%s: FOTA_proc_write %d \n", __func__, cmd);

    switch(cmd){
        case '0':
            ifx_power_low();
            break;
        case '1':
            ifx_power_high();
            break;
        case '2':
            ifx_reset_low();
            break;
        case '3':
            ifx_reset_high();
            break;
        case '4':
	    ifx_uart_sw_ctrl();
            break;
        case '5':
	    ifx_fota_reset();
            break;	    
        case '6':
        	mdm_reset();
        	break;

#if defined(CONFIG_KS1001) ||defined(CONFIG_KS1103) || defined(CONFIG_LU6500)
        case '7':
        	CTRL_GPIO_high();
        	break;
        case '8':
        	CTRL_GPIO_low();
        	break;
//                                                    
        case '9':
        	printk("fota reset delayed work run\n");
        	queue_delayed_work(system_nrt_wq, &work, msecs_to_jiffies(20000));
        	break;
//                                                    
#endif

	default :
            printk("FOTA Driver invalid arg\n");
            break;
    }
    return len;
}

static struct file_operations star_fota_test_proc_ops = 
{
    .read = fota_test_proc_read,
    .write = fota_test_proc_write,
};

int create_star_fota_test_proc_file(void)
{
    struct proc_dir_entry *star_fota_test_proc_file = NULL;
    star_fota_test_proc_file = create_proc_entry(STAR_ERS_TEST_PROC_FILE, 0777, NULL);
    
    if (star_fota_test_proc_file) 
    {
        star_fota_test_proc_file->proc_fops = &star_fota_test_proc_ops;
    } 
    else
    { 
        printk(KERN_INFO "LGE: Star fota_test proc file create failed!\n");
    }

	INIT_DELAYED_WORK(&work, fota_reset_worker); //                                           
    return 0;
}

void remove_star_fota_test_proc_file(void)
{
	cancel_delayed_work_sync(&work); //                                           
    remove_proc_entry(STAR_ERS_TEST_PROC_FILE, NULL);
}

static int __init star_fota_test_init(void)
{
    return create_star_fota_test_proc_file();
}

static void __exit star_fota_test_exit(void)
{
    remove_star_fota_test_proc_file();
}

module_init(star_fota_test_init);
module_exit(star_fota_test_exit);

MODULE_AUTHOR("LG Electronics");
MODULE_DESCRIPTION("Star ERS Test Driver");
MODULE_LICENSE("GPL");
