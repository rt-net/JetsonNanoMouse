// SPDX-License-Identifier: GPL-2.0-only
/*
 *
 * rtmouse.c
 * Jetson Nano Mouse device driver
 *
 * Version: 0.1.5
 *
 * Copyright (C) 2019-2020 RT Corporation <shop@rt-net.jp>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 *
 */

#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>

MODULE_AUTHOR("RT Corporation");
MODULE_DESCRIPTION("A device driver of Jetson Nano Mouse");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1.6");

/* --- Options --- */
#define ALTERNATIVE_SPI_CS
//#define USE_EXTERNAL_CLOCK

/* --- GPIO Pins --- */
#define GPIO_LED0 13	     // PIN22
#define GPIO_LED1 15	     // PIN18
#define GPIO_LED2 232	     // PIN16
#define GPIO_LED3 79	     // PIN16
#define GPIO_SW0 78	     // PIN40
#define GPIO_SW1 12	     // PIN37
#define GPIO_SW2 77	     // PIN38
#define GPIO_SEN_R 194	     // PIN15
#define GPIO_SEN_L 149	     // PIN29
#define GPIO_SEN_RF 14	     // PI13
#define GPIO_SEN_LF 50	     // PIN11
#define GPIO_MOTOR_EN 200    // PIN31
#define GPIO_MOTOR_DIR_R 168 // PIN32
#define GPIO_MOTOR_DIR_L 216 // PIN7

#define GPIO_SPI_CS 19 // PIN24

#define MAX_BUFLEN 64
#define DEBOUNCE_TIME 50

#define DEV_MAJOR 0
#define DEV_MINOR 0

#define REG_GPIO_NAME "Jetson Nano GPIO"

#define NUM_DEV_LED 4
#define NUM_DEV_SWITCH 3
#define NUM_DEV_SWITCHES 1
#define NUM_DEV_SENSOR 1
#define NUM_DEV_MOTOREN 1
#define NUM_DEV_MOTOR 1
#define NUM_DEV_TOTAL                                                          \
	(NUM_DEV_LED + NUM_DEV_SWITCH + NUM_DEV_SWITCHES + NUM_DEV_SENSOR +    \
	 NUM_DEV_MOTOREN + NUM_DEV_MOTOR)

#define NUM_DEV_CNTR 1
#define NUM_DEV_CNTL 1
#define NUM_DEV_RTCNT_TOTAL (NUM_DEV_CNTR + NUM_DEV_CNTL)

#define NUM_DEV_BUZZER 1
#define NUM_DEV_MOTORRAWR 1
#define NUM_DEV_MOTORRAWL 1

#define NUM_DEV_PWM_TOTAL                                                      \
	(NUM_DEV_BUZZER + NUM_DEV_MOTORRAWR + NUM_DEV_MOTORRAWL)

/* I2C devices */
#define NUM_DEV_CNT 1

#define DRIVER_NAME "rtmouse"

#define DEVNAME_LED "rtled"
#define DEVNAME_SWITCH "rtswitch"
#define DEVNAME_SWITCHES "rtswitches"
#define DEVNAME_SENSOR "rtlightsensor"
#define DEVNAME_BUZZER "rtbuzzer"
#define DEVNAME_MOTOREN "rtmotoren"
#define DEVNAME_MOTOR "rtmotor"
#define DEVNAME_CNTR "rtcounter_r"
#define DEVNAME_CNTL "rtcounter_l"
#define DEVNAME_MOTORRAWR "rtmotor_raw_r"
#define DEVNAME_MOTORRAWL "rtmotor_raw_l"

static struct cdev *cdev_array = NULL;
static struct cdev *cdev_pwm_array = NULL;
static struct class *class_led = NULL;
static struct class *class_switch = NULL;
static struct class *class_switches = NULL;
static struct class *class_sensor = NULL;
static struct class *class_motoren = NULL;
static struct class *class_motor = NULL;
static struct class *class_buzzer = NULL;
static struct class *class_motorrawr = NULL;
static struct class *class_motorrawl = NULL;

static volatile int cdev_index = 0;
static volatile int cdev_pwm_index = 0;

static struct mutex lock;

static int _major_led = DEV_MAJOR;
static int _minor_led = DEV_MINOR;

static int _major_switch = DEV_MAJOR;
static int _minor_switch = DEV_MINOR;

static int _major_switches = DEV_MAJOR;
static int _minor_switches = DEV_MINOR;

static int _major_sensor = DEV_MAJOR;
static int _minor_sensor = DEV_MINOR;

static int _major_motoren = DEV_MAJOR;
static int _minor_motoren = DEV_MINOR;

static int _major_motor = DEV_MAJOR;
static int _minor_motor = DEV_MINOR;

static int _major_buzzer = DEV_MAJOR;
static int _minor_buzzer = DEV_MINOR;

static int _major_motorrawr = DEV_MAJOR;
static int _minor_motorrawr = DEV_MINOR;

static int _major_motorrawl = DEV_MAJOR;
static int _minor_motorrawl = DEV_MINOR;

/* SPI Parameters */
static int spi_bus_num = 0;

#ifndef ALTERNATIVE_SPI_CS
static int spi_chip_select = 0;
#endif
#ifdef ALTERNATIVE_SPI_CS
// To work around the problem of SPI_CS not working properly.
static int spi_chip_select = 1;
#endif

/* --- A/D Parameters --- */
#define MCP320X_PACKET_SIZE 3
#define MCP320X_DIFF 0
#define MCP320X_SINGLE 1
#define MCP3204_CHANNELS 4
/* --- A/D Channels --- */
#define R_AD_CH 3
#define L_AD_CH 0
#define RF_AD_CH 2
#define LF_AD_CH 1

/* I2C Parameters */
#define CNTL_I2C_ADDR 0x10
#define CNTR_I2C_ADDR 0x11
#define CNT_MSB_REG 0x10
#define CNT_LSB_REG 0x11
#define PCA9685_L_I2C_ADDR 0x40
#define PCA9685_R_I2C_ADDR 0x41
#define PCA9685_MODE1 0x00
#define PCA9685_MODE2 0x01
#define PCA9685_PRESCALE 0xFE
#define PCA9685_LED0_ON_L 0x06
#define PCA9685_LED0_ON_H 0x07
#define PCA9685_LED0_OFF_L 0x08
#define PCA9685_LED0_OFF_H 0x09
#define PCA9685_ALL_LED_ON_L 0xFA
#define PCA9685_ALL_LED_ON_H 0xFB
#define PCA9685_ALL_LED_OFF_L 0xFC
#define PCA9685_ALL_LED_OFF_H 0xFD
#define PCA9685_RESTART 0x80
#define PCA9685_SLEEP 0x10
#define PCA9685_ALLCALL 0x01
#define PCA9685_OUTDRV 0x04

#ifdef USE_EXTERNAL_CLOCK
#define PCA9685_CLOCK 0x40
#endif
#ifndef USE_EXTERNAL_CLOCK
#define PCA9685_CLOCK 0x00
#endif

/* Motor Parameter */
#define MOTOR_UNCONTROLLABLE_FREQ 5

/* --- Function Declarations --- */
static void set_motor_r_freq(int freq);
static void set_motor_l_freq(int freq);
static int mcp3204_remove(struct spi_device *spi);
static int mcp3204_probe(struct spi_device *spi);
static unsigned int mcp3204_get_value(int channel);
static int i2c_rtcnt_probe(struct i2c_client *client,
			   const struct i2c_device_id *id);
static int i2c_rtcnt_remove(struct i2c_client *client);
static int i2c_pwm_probe(struct i2c_client *client,
			 const struct i2c_device_id *id);
static int i2c_pwm_remove(struct i2c_client *client);

static ssize_t pwm_motorr_write_core(int pwm_freq);
static ssize_t pwm_motorl_write_core(int pwm_freq);

/* --- Variable Type definitions --- */
/* SPI */
struct mcp3204_drvdata {
	struct spi_device *spi;
	struct mutex lock;
	unsigned char tx[MCP320X_PACKET_SIZE] ____cacheline_aligned;
	unsigned char rx[MCP320X_PACKET_SIZE] ____cacheline_aligned;
	struct spi_transfer xfer ____cacheline_aligned;
	struct spi_message msg ____cacheline_aligned;
};

/* I2C */
struct i2c_device_info {
	struct cdev cdev;
	unsigned int device_major;
	struct class *device_class;
	struct i2c_client *client;
	struct mutex lock;
};

/* --- Static variables --- */
/* SPI device ID */
static struct spi_device_id mcp3204_id[] = {
    {"mcp3204", 0},
    {},
};

/* SPI Info */
static struct spi_board_info mcp3204_info = {
    .modalias = "mcp3204",
    .max_speed_hz = 100000,
    .bus_num = 0,
    .chip_select = 0,
    .mode = SPI_MODE_3,
};

/* SPI Dirver Info */
static struct spi_driver mcp3204_driver = {
    .driver =
	{
	    .name = DEVNAME_SENSOR,
	    .owner = THIS_MODULE,
	},
    .id_table = mcp3204_id,
    .probe = mcp3204_probe,
    .remove = mcp3204_remove,
};

/* I2C client */
static struct i2c_client *i2c_client_r = NULL;
static struct i2c_client *i2c_client_l = NULL;
static struct i2c_client *i2c_client_pwm0 = NULL;
static struct i2c_client *i2c_client_pwm1 = NULL;

/* I2C Device ID */
static struct i2c_device_id i2c_rtcnt_id[] = {
    {DEVNAME_CNTL, 0},
    {DEVNAME_CNTR, 1},
    {},
};

static struct i2c_device_id i2c_pwm_id[] = {
    {"pwmdriver0", 0},
    {"pwmdriver1", 1},
    {},
};

/* I2C Dirver Info */
static struct i2c_driver i2c_counter_driver = {
    .driver =
	{
	    .name = "rtcounter",
	    .owner = THIS_MODULE,
	},
    .id_table = i2c_rtcnt_id,
    .probe = i2c_rtcnt_probe,
    .remove = i2c_rtcnt_remove,
};

static struct i2c_driver i2c_pwm_driver = {
    .driver =
	{
	    .name = "pwmdriver",
	    .owner = THIS_MODULE,
	},
    .id_table = i2c_pwm_id,
    .probe = i2c_pwm_probe,
    .remove = i2c_pwm_remove,
};

static struct i2c_device_info *i2c_pwm0_dev_info;
static struct i2c_device_info *i2c_pwm1_dev_info;

/* -- Device Addition -- */
MODULE_DEVICE_TABLE(spi, mcp3204_id);
MODULE_DEVICE_TABLE(i2c, i2c_rtcnt_id);
MODULE_DEVICE_TABLE(i2c, i2c_pwm_id);

/*
 * Turn On LEDs
 * return 0 : device close
 */
static int led_put(int ledno)
{
	switch (ledno) {
	case 0:
		gpio_set_value(GPIO_LED0, 1);
		break;
	case 1:
		gpio_set_value(GPIO_LED1, 1);
		break;
	case 2:
		gpio_set_value(GPIO_LED2, 1);
		break;
	case 3:
		gpio_set_value(GPIO_LED3, 1);
		break;
	}
	return 0;
}

/*
 * Turn Off LEDs
 * return 0 : device close
 */
static int led_del(int ledno)
{
	switch (ledno) {
	case 0:
		gpio_set_value(GPIO_LED0, 0);
		break;
	case 1:
		gpio_set_value(GPIO_LED1, 0);
		break;
	case 2:
		gpio_set_value(GPIO_LED2, 0);
		break;
	case 3:
		gpio_set_value(GPIO_LED3, 0);
		break;
	}

	return 0;
}

/* left motor function */
static void set_motor_l_freq(int freq)
{
	pwm_motorl_write_core(freq);
	return;
}

/* right motor function */
static void set_motor_r_freq(int freq)
{
	pwm_motorr_write_core(freq);
	return;
}

/* --- Function for device file operations --- */
/* Parse motor command  */
static int parseMotorCmd(const char __user *buf, size_t count, int *ret)
{
	int r_motor_val, l_motor_val, time_val;
	char *newbuf = kmalloc(sizeof(char) * count, GFP_KERNEL);

	if (copy_from_user(newbuf, buf, sizeof(char) * count)) {
		kfree(newbuf);
		return -EFAULT;
	}

	sscanf(newbuf, "%d%d%d\n", &l_motor_val, &r_motor_val, &time_val);

	kfree(newbuf);

	mutex_lock(&lock);

	set_motor_l_freq(l_motor_val);
	set_motor_r_freq(r_motor_val);

	msleep_interruptible(time_val);

	set_motor_l_freq(0);
	set_motor_r_freq(0);

	mutex_unlock(&lock);

	return count;
}

/*
 * led_write - Trun ON/OFF LEDs
 * Write function of /dev/rtled
 */
static ssize_t led_write(struct file *filep, const char __user *buf,
			 size_t count, loff_t *f_pos)
{
	char cval;
	int ret;
	int minor = *((int *)filep->private_data);

	if (count > 0) {
		if (copy_from_user(&cval, buf, sizeof(char))) {
			return -EFAULT;
		}
		switch (cval) {
		case '1':
			ret = led_put(minor);
			break;
		case '0':
			ret = led_del(minor);
			break;
		}
		return sizeof(char);
	}
	return 0;
}

/*
 * Read Push Switches
 * return 0 : device close
 */
static ssize_t sw_read(struct file *filep, char __user *buf, size_t count,
		       loff_t *f_pos)
{
	int buflen = 0;
	unsigned char rw_buf[MAX_BUFLEN];
	unsigned int ret = 0;
	int len;
	unsigned int pin = GPIO_SW0;
	int minor = *((int *)filep->private_data);

	switch (minor) {
	case 0:
		pin = GPIO_SW0;
		break;
	case 1:
		pin = GPIO_SW1;
		break;
	case 2:
		pin = GPIO_SW2;
		break;
	default:
		return 0;
		break;
	}

	if (*f_pos > 0)
		return 0; /* End of file */

	ret = !gpio_get_value(pin);
	sprintf(rw_buf, "%d\n", ret);

	buflen = strlen(rw_buf);
	count = buflen;
	len = buflen;

	if (copy_to_user((void *)buf, &rw_buf, count)) {
		printk(KERN_INFO "err read buffer from ret  %d\n", ret);
		printk(KERN_INFO "err read buffer from %s\n", rw_buf);
		printk(KERN_INFO "err sample_char_read size(%ld)\n", count);
		printk(KERN_INFO "sample_char_read size err(%d)\n", -EFAULT);
		return 0;
	}
	*f_pos += count;

	return count;
}

/*
 * Read All Push Switches
 * return 0 : device close
 */
static ssize_t sws_read(struct file *filep, char __user *buf, size_t count,
			loff_t *f_pos)
{
	int buflen = 0;
	unsigned char rw_buf[MAX_BUFLEN];
	unsigned int ret = 0;
	int len;

	if (*f_pos > 0)
		return 0; /* End of file */

	sprintf(rw_buf, "%d %d %d\n", !gpio_get_value(GPIO_SW0),
		!gpio_get_value(GPIO_SW1), !gpio_get_value(GPIO_SW2));

	buflen = strlen(rw_buf);
	count = buflen;
	len = buflen;

	if (copy_to_user((void *)buf, &rw_buf, count)) {
		printk(KERN_INFO "err read buffer from ret  %d\n", ret);
		printk(KERN_INFO "err read buffer from %s\n", rw_buf);
		printk(KERN_INFO "err sample_char_read size(%ld)\n", count);
		printk(KERN_INFO "sample_char_read size err(%d)\n", -EFAULT);
		return 0;
	}
	*f_pos += count;

	return count;
}

/*
 * Read Sensor information
 * return 0 : device close
 */
static ssize_t sensor_read(struct file *filep, char __user *buf, size_t count,
			   loff_t *f_pos)
{
	int buflen = 0;
	unsigned char rw_buf[MAX_BUFLEN];
	unsigned int ret = 0;
	int len;

	int usecs = 30;
	int rf = 0, lf = 0, r = 0, l = 0;
	int orf = 0, olf = 0, or = 0, ol = 0;

	if (*f_pos > 0)
		return 0; /* End of file */

#ifndef ALTERNATIVE_SPI_CS
	/* get values through MCP3204 */
	/* Right side */
	or = mcp3204_get_value(R_AD_CH);
	gpio_set_value(GPIO_SEN_R, 1);
	udelay(usecs);
	r = mcp3204_get_value(R_AD_CH);
	gpio_set_value(GPIO_SEN_R, 0);
	udelay(usecs);
	/* Left side */
	ol = mcp3204_get_value(L_AD_CH);
	gpio_set_value(GPIO_SEN_L, 1);
	udelay(usecs);
	l = mcp3204_get_value(L_AD_CH);
	gpio_set_value(GPIO_SEN_L, 0);
	udelay(usecs);
	/* Right front side */
	orf = mcp3204_get_value(RF_AD_CH);
	gpio_set_value(GPIO_SEN_RF, 1);
	udelay(usecs);
	rf = mcp3204_get_value(RF_AD_CH);
	gpio_set_value(GPIO_SEN_RF, 0);
	udelay(usecs);
	/* Left front side */
	olf = mcp3204_get_value(LF_AD_CH);
	gpio_set_value(GPIO_SEN_LF, 1);
	udelay(usecs);
	lf = mcp3204_get_value(LF_AD_CH);
	gpio_set_value(GPIO_SEN_LF, 0);
	udelay(usecs);
#endif
#ifdef ALTERNATIVE_SPI_CS
	/* get values through MCP3204 */
	/* Right side */
	gpio_set_value(GPIO_SPI_CS, 0);
	or = mcp3204_get_value(R_AD_CH);
	gpio_set_value(GPIO_SPI_CS, 1);
	gpio_set_value(GPIO_SEN_R, 1);
	udelay(usecs);
	gpio_set_value(GPIO_SPI_CS, 0);
	r = mcp3204_get_value(R_AD_CH);
	gpio_set_value(GPIO_SPI_CS, 1);
	gpio_set_value(GPIO_SEN_R, 0);
	udelay(usecs);
	/* Left side */
	gpio_set_value(GPIO_SPI_CS, 0);
	ol = mcp3204_get_value(L_AD_CH);
	gpio_set_value(GPIO_SPI_CS, 1);
	gpio_set_value(GPIO_SEN_L, 1);
	udelay(usecs);
	gpio_set_value(GPIO_SPI_CS, 0);
	l = mcp3204_get_value(L_AD_CH);
	gpio_set_value(GPIO_SPI_CS, 1);
	gpio_set_value(GPIO_SEN_L, 0);
	udelay(usecs);
	/* Right front side */
	gpio_set_value(GPIO_SPI_CS, 0);
	orf = mcp3204_get_value(RF_AD_CH);
	gpio_set_value(GPIO_SPI_CS, 1);
	gpio_set_value(GPIO_SEN_RF, 1);
	udelay(usecs);
	gpio_set_value(GPIO_SPI_CS, 0);
	rf = mcp3204_get_value(RF_AD_CH);
	gpio_set_value(GPIO_SPI_CS, 1);
	gpio_set_value(GPIO_SEN_RF, 0);
	udelay(usecs);
	/* Left front side */
	gpio_set_value(GPIO_SPI_CS, 0);
	olf = mcp3204_get_value(LF_AD_CH);
	gpio_set_value(GPIO_SPI_CS, 1);
	gpio_set_value(GPIO_SEN_LF, 1);
	udelay(usecs);
	gpio_set_value(GPIO_SPI_CS, 0);
	lf = mcp3204_get_value(LF_AD_CH);
	gpio_set_value(GPIO_SPI_CS, 1);
	gpio_set_value(GPIO_SEN_LF, 0);
	udelay(usecs);
#endif

	/* set sensor data to rw_buf(static buffer) */
	snprintf(rw_buf, sizeof(rw_buf), "%d %d %d %d\n", rf - orf, r - or,
		 l - ol, lf - olf);
	buflen = strlen(rw_buf);
	count = buflen;
	len = buflen;

	/* copy data to user area */
	if (copy_to_user((void *)buf, &rw_buf, count)) {
		printk(KERN_INFO "err read buffer from ret  %d\n", ret);
		printk(KERN_INFO "err read buffer from %s\n", rw_buf);
		printk(KERN_INFO "err sample_char_read size(%ld)\n", count);
		printk(KERN_INFO "sample_char_read size err(%d)\n", -EFAULT);
		return 0;
	}

	*f_pos += count;

	return count;
}

/*
 * motoren_write - Trun ON/OFF motor enable signal
 * Write function of /dev/rtmotoren
 */
static ssize_t motoren_write(struct file *filep, const char __user *buf,
			     size_t count, loff_t *f_pos)
{
	char cval;

	if (count > 0) {
		if (copy_from_user(&cval, buf, sizeof(char))) {
			return -EFAULT;
		}
		switch (cval) {
		case '1':
			gpio_set_value(GPIO_MOTOR_EN, 1);
			break;
		case '0':
			gpio_set_value(GPIO_MOTOR_EN, 0);
			break;
		}
		return sizeof(char);
	}
	return 0;
}

/*
 *  motor_write - Output frequency to right and left both motors
 *  Write function of /dev/rtmotor
 */
static ssize_t motor_write(struct file *filep, const char __user *buf,
			   size_t count, loff_t *f_pos)
{
	int tmp;
	int bufcnt;
	bufcnt = parseMotorCmd(buf, count, &tmp);

	return bufcnt;
}

/*
 * i2c_counter_set - set value to I2C pulse counter
 * called by cntr_write() and cntl_write()
 */
static int i2c_counter_set(struct i2c_device_info *dev_info, int setval)
{
	int ret = 0;
	int lsb = 0, msb = 0;
	struct i2c_client *client = dev_info->client;

	// printk(KERN_INFO "set 0x%x = 0x%x\n", client->addr, setval);
	msb = (setval >> 8) & 0xFF;
	lsb = setval & 0xFF;
	mutex_lock(&dev_info->lock);
	// printk(KERN_INFO "set 0x%x msb = 0x%x\n", client->addr, msb);
	ret = i2c_smbus_write_byte_data(client, CNT_MSB_REG, msb);
	if (ret < 0) {
		printk(KERN_ERR
		       "%s: Failed writing to i2c counter device, addr=0x%x\n",
		       __func__, client->addr);
		return -ENODEV;
	}
	// printk(KERN_INFO "set 0x%x lsb = 0x%x\n", client->addr, lsb);
	ret = i2c_smbus_write_byte_data(client, CNT_LSB_REG, lsb);
	if (ret < 0) {
		printk(KERN_ERR
		       "%s: Failed writing to i2c counter device, addr=0x%x\n",
		       __func__, client->addr);
		return -ENODEV;
	}
	mutex_unlock(&dev_info->lock);
	return ret;
}

/*
 * i2c_counter_read - get value from I2C pulse counter
 * called by rtcnt_read()
 */
static int i2c_counter_read(struct i2c_device_info *dev_info, int *ret)
{
	int lsb = 0, msb = 0;
	// printk(KERN_INFO "read 0x%x\n", client->addr);
	struct i2c_client *client = dev_info->client;
	mutex_lock(&dev_info->lock);

	lsb = i2c_smbus_read_byte_data(client, CNT_LSB_REG);
	if (lsb < 0) {
		printk(
		    KERN_ERR
		    "%s: Failed reading from i2c counter device, addr=0x%x\n",
		    __func__, client->addr);
		return -ENODEV;
	}
	msb = i2c_smbus_read_byte_data(client, CNT_MSB_REG);
	if (msb < 0) {
		printk(
		    KERN_ERR
		    "%s: Failed reading from i2c counter device, addr=0x%x\n",
		    __func__, client->addr);
		return -ENODEV;
	}
	mutex_unlock(&dev_info->lock);

	*ret = ((msb << 8) & 0xFF00) + (lsb & 0xFF);

	// printk(KERN_INFO "0x%x == 0x%x\n", client->addr, *ret);
	return 0;
}

/*
 *  rtcnt_read - Read value from right/left pulse counter
 *  Read function of /dev/rtcounter_*
 */
static ssize_t rtcnt_read(struct file *filep, char __user *buf, size_t count,
			  loff_t *pos)
{
	struct i2c_device_info *dev_info = filep->private_data;

	unsigned char rw_buf[64];
	int buflen;

	int rtcnt_count = 0;
	if (*pos > 0)
		return 0; /* close device */
	i2c_counter_read(dev_info, &rtcnt_count);

	/* set sensor data to rw_buf(static buffer) */
	sprintf(rw_buf, "%d\n", rtcnt_count);
	buflen = strlen(rw_buf);
	count = buflen;

	/* copy data to user area */
	if (copy_to_user((void *)buf, &rw_buf, count)) {
		printk(KERN_INFO "err read buffer from %s\n", rw_buf);
		printk(KERN_INFO "err sample_char_read size(%ld)\n", count);
		printk(KERN_INFO "sample_char_read size err(%d)\n", -EFAULT);
		return -EFAULT;
	}
	*pos += count;
	return count;
}

/*
 *  rtcnt_write - Set value to right/left pulse counter
 *  Write function of /dev/rtcounter
 */
static ssize_t rtcnt_write(struct file *filep, const char __user *buf,
			   size_t count, loff_t *pos)
{
	struct i2c_device_info *dev_info = filep->private_data;

	int ret = -1;
	int counter_set_value = 0;

	if (count < 0)
		return 0;

	ret = kstrtoint_from_user(buf, count, 10, &counter_set_value);
	if (ret) {
		printk(KERN_ERR "%s: error parsing string to int in %s()\n",
		       DRIVER_NAME, __func__);
		return ret;
	}

	i2c_counter_set(dev_info, counter_set_value);

	printk(KERN_DEBUG "%s: set pulse counter value %d\n", DRIVER_NAME,
	       counter_set_value);
	return count;
}

/*
 * i2c_pwm_set_freq - set freq of I2C pwm driver
 * called by pwm_*_write()
 */
static int i2c_pwm_set_freq(struct i2c_device_info *dev_info, int freq)
{
	int ret = 0;
	struct i2c_client *client = dev_info->client;
	int prescale = 0;
	int oldmode = 0;

	printk(KERN_DEBUG "%s: set 0x%x to %d[Hz]\n", DRIVER_NAME, client->addr,
	       freq);

	mutex_lock(&dev_info->lock);

	/* get present mode */
	oldmode = i2c_smbus_read_byte_data(client, PCA9685_MODE1);
	if (oldmode < 0) {
		printk(KERN_ERR
		       "%s: Failed reading from i2c pwm driver, addr=0x%x\n",
		       __func__, client->addr);
		mutex_unlock(&dev_info->lock);
		return -ENODEV;
	}
	printk(KERN_DEBUG "%s: PCA9685 MODE1 %d\n", DRIVER_NAME, oldmode);

	/* set device sleep */
	ret = i2c_smbus_write_byte_data(client, PCA9685_MODE1,
					(oldmode & ~PCA9685_RESTART) |
					    PCA9685_SLEEP);
	if (ret < 0) {
		printk(KERN_ERR
		       "%s: Failed writing to i2c counter device, addr=0x%x\n",
		       __func__, client->addr);
		mutex_unlock(&dev_info->lock);
		return -ENODEV;
	}
#ifdef USE_EXTERNAL_CLOCK
	/* set external clock */
	ret = i2c_smbus_write_byte_data(client, PCA9685_MODE1,
					(oldmode & ~PCA9685_RESTART) |
					    PCA9685_SLEEP | PCA9685_CLOCK);
	if (ret < 0) {
		printk(KERN_ERR
		       "%s: Failed writing to i2c counter device, addr=0x%x\n",
		       __func__, client->addr);
		mutex_unlock(&dev_info->lock);
		return -ENODEV;
	}
#endif
	/* set prescale */
	// (int)(round(25*10^6/4096/freq) - 1
	if ((25000000 * 10 / 4096 / freq) % 10 < 5) {
		prescale = (int)(25000000 / 4096 / freq) - 1;
	} else {
		prescale = (int)(25000000 / 4096 / freq);
	}
	ret = i2c_smbus_write_byte_data(client, PCA9685_PRESCALE, prescale);
	if (ret < 0) {
		printk(KERN_ERR
		       "%s: Failed writing to i2c counter device, addr=0x%x\n",
		       __func__, client->addr);
		mutex_unlock(&dev_info->lock);
		return -ENODEV;
	}
	/* get device awake */
	ret = i2c_smbus_write_byte_data(client, PCA9685_MODE1,
					(oldmode & ~PCA9685_RESTART) |
					    PCA9685_CLOCK);

	if (ret < 0) {
		printk(KERN_ERR
		       "%s: Failed writing to i2c counter device, addr=0x%x\n",
		       __func__, client->addr);
		mutex_unlock(&dev_info->lock);
		return -ENODEV;
	}
	udelay(500);
	ret = i2c_smbus_write_byte_data(
	    client, PCA9685_MODE1, oldmode | PCA9685_RESTART | PCA9685_CLOCK);
	if (ret < 0) {
		printk(KERN_ERR
		       "%s: Failed writing to i2c counter device, addr=0x%x\n",
		       __func__, client->addr);
		mutex_unlock(&dev_info->lock);
		return -ENODEV;
	}
	mutex_unlock(&dev_info->lock);
	return ret;
}

/*
 * i2c_pwm_set_all - set freq of I2C pwm driver
 * called by pwm_*_write()
 */
static int i2c_pwm_set_all(struct i2c_device_info *dev_info, int on, int off)
{
	int ret = 0;
	struct i2c_client *client = dev_info->client;

	mutex_lock(&dev_info->lock);
	// printk(KERN_INFO "set 0x%x msb = 0x%x\n", client->addr, msb);
	/* set all 0 */
	ret =
	    i2c_smbus_write_byte_data(client, PCA9685_ALL_LED_ON_L, on & 0xFF);
	if (ret < 0) {
		printk(KERN_ERR "%s: Failed writing to i2c counter device, "
				"ALL_LED_ON_L, addr=0x%x\n",
		       __func__, client->addr);
		mutex_unlock(&dev_info->lock);
		return -ENODEV;
	}
	ret = i2c_smbus_write_byte_data(client, PCA9685_ALL_LED_ON_H, on >> 8);
	if (ret < 0) {
		printk(KERN_ERR "%s: Failed writing to i2c counter device, "
				"ALL_LED_ON_H, addr=0x%x\n",
		       __func__, client->addr);
		mutex_unlock(&dev_info->lock);
		return -ENODEV;
	}
	ret = i2c_smbus_write_byte_data(client, PCA9685_ALL_LED_OFF_H,
					off & 0xFF);
	if (ret < 0) {
		printk(KERN_ERR "%s: Failed writing to i2c counter device, "
				"ALL_LED_OFF_H, addr=0x%x\n",
		       __func__, client->addr);
		mutex_unlock(&dev_info->lock);
		return -ENODEV;
	}
	ret =
	    i2c_smbus_write_byte_data(client, PCA9685_ALL_LED_OFF_L, off >> 8);
	if (ret < 0) {
		printk(KERN_ERR "%s: Failed writing to i2c counter device, "
				"ALL_LED_OFF_L, addr=0x%x\n",
		       __func__, client->addr);
		mutex_unlock(&dev_info->lock);
		return -ENODEV;
	}
	mutex_unlock(&dev_info->lock);
	return ret;
}

/*
 * i2c_pwm_set - set freq of I2C pwm driver
 * called by pwm_*_write()
 */
static int i2c_pwm_set(struct i2c_device_info *dev_info, int channel, int on,
		       int off)
{
	int ret = 0;
	struct i2c_client *client = dev_info->client;

	printk(KERN_DEBUG "set 0x%x: 'channel: %d, on: %d, off: %d'\n",
	       client->addr, channel, on, off);

	mutex_lock(&dev_info->lock);
	// printk(KERN_INFO "set 0x%x msb = 0x%x\n", client->addr, msb);
	ret = i2c_smbus_write_byte_data(client, PCA9685_LED0_ON_L + 4 * channel,
					on & 0xFF);
	if (ret < 0) {
		printk(KERN_ERR
		       "%s: Failed writing to i2c counter device, addr=0x%x\n",
		       __func__, client->addr);
		mutex_unlock(&dev_info->lock);
		return -ENODEV;
	}
	ret = i2c_smbus_write_byte_data(client, PCA9685_LED0_ON_H + 4 * channel,
					on >> 8);
	if (ret < 0) {
		printk(KERN_ERR
		       "%s: Failed writing to i2c counter device, addr=0x%x\n",
		       __func__, client->addr);
		mutex_unlock(&dev_info->lock);
		return -ENODEV;
	}
	ret = i2c_smbus_write_byte_data(
	    client, PCA9685_LED0_OFF_L + 4 * channel, off & 0xFF);
	if (ret < 0) {
		printk(KERN_ERR
		       "%s: Failed writing to i2c counter device, addr=0x%x\n",
		       __func__, client->addr);
		mutex_unlock(&dev_info->lock);
		return -ENODEV;
	}
	ret = i2c_smbus_write_byte_data(
	    client, PCA9685_LED0_OFF_H + 4 * channel, off >> 8);
	if (ret < 0) {
		printk(KERN_ERR
		       "%s: Failed writing to i2c counter device, addr=0x%x\n",
		       __func__, client->addr);
		mutex_unlock(&dev_info->lock);
		return -ENODEV;
	}
	mutex_unlock(&dev_info->lock);
	return ret;
}

/*
 *  pwm_buzzer_write - Set value to pwm driver
 *  Write function of /dev/rtbuzzer
 */
static ssize_t pwm_buzzer_write(struct file *filep, const char __user *buf,
				size_t count, loff_t *pos)
{
	// struct i2c_device_info *dev_info = filep->private_data;
	struct i2c_device_info *dev_info = i2c_pwm1_dev_info;

	int ret = -1;
	int pwm_freq = 0;
	static int previous_pwm_freq = 0;

	if (count < 0)
		return 0;

	ret = kstrtoint_from_user(buf, count, 10, &pwm_freq);
	if (ret) {
		printk(KERN_ERR "%s: error parsing string to int in %s()\n",
		       DRIVER_NAME, __func__);
		return ret;
	}

	if (pwm_freq != previous_pwm_freq) {
		// i2c_pwm_set_all(dev_info, 0, 0);
		if (pwm_freq) {
			ret = i2c_pwm_set_freq(dev_info, pwm_freq);
			if (ret) {
				printk(KERN_ERR
				       "%s: error i2c_pwm_set_freq in %s()\n",
				       DRIVER_NAME, __func__);
				return ret;
			}
			ret = i2c_pwm_set(dev_info, 1, 0, 3072);
			if (ret) {
				printk(KERN_ERR
				       "%s: error i2c_pwm_set in %s()\n",
				       DRIVER_NAME, __func__);
				return ret;
			}
		} else {
			ret = i2c_pwm_set(dev_info, 1, 0, 0);
			if (ret) {
				printk(KERN_ERR
				       "%s: error i2c_pwm_set in %s()\n",
				       DRIVER_NAME, __func__);
				return ret;
			}
		}
		printk(KERN_DEBUG "%s: set pwm driver freq %d\n", DRIVER_NAME,
		       pwm_freq);
	}
	previous_pwm_freq = pwm_freq;
	return count;
}

/*
 *  pwm_motorr_write - Set value to pwm driver
 *  Write function of /dev/rtmotor_raw_r
 */
static ssize_t pwm_motorr_write(struct file *filep, const char __user *buf,
				size_t count, loff_t *pos)
{
	int ret = -1;
	int pwm_freq = 0;

	if (count < 0)
		return 0;

	ret = kstrtoint_from_user(buf, count, 10, &pwm_freq);
	if (ret) {
		printk(KERN_ERR "%s: error parsing string to int in %s()\n",
		       DRIVER_NAME, __func__);
		return ret;
	}

	if (pwm_freq < 0) {
		gpio_set_value(GPIO_MOTOR_DIR_R, 0);
		pwm_freq *= -1;
	} else {
		gpio_set_value(GPIO_MOTOR_DIR_R, 1);
	}

	ret = pwm_motorr_write_core(pwm_freq);
	if (ret < 0)
		return ret;
	return count;
}

static ssize_t pwm_motorr_write_core(int pwm_freq)
{
	// struct i2c_device_info *dev_info = filep->private_data;
	struct i2c_device_info *dev_info = i2c_pwm1_dev_info;

	int ret = 0;
	static int previous_pwm_freq = 0;

	if (pwm_freq < 0) {
		gpio_set_value(GPIO_MOTOR_DIR_R, 0);
		pwm_freq *= -1;
	} else {
		gpio_set_value(GPIO_MOTOR_DIR_R, 1);
	}

	if (pwm_freq != previous_pwm_freq) {
		// i2c_pwm_set_all(dev_info, 0, 0);
		if (pwm_freq) {
			ret = i2c_pwm_set_freq(dev_info, pwm_freq);
			if (ret) {
				printk(KERN_ERR
				       "%s: error i2c_pwm_set_freq in %s()\n",
				       DRIVER_NAME, __func__);
				return ret;
			}
			ret = i2c_pwm_set(dev_info, 0, 0, 2048);
			if (ret) {
				printk(KERN_ERR
				       "%s: error i2c_pwm_set in %s()\n",
				       DRIVER_NAME, __func__);
				return ret;
			}
		} else {
			ret = i2c_pwm_set(dev_info, 0, 0, 0);
			if (ret) {
				printk(KERN_ERR
				       "%s: error i2c_pwm_set in %s()\n",
				       DRIVER_NAME, __func__);
				return ret;
			}
		}
		printk(KERN_DEBUG "%s: set pwm driver freq %d\n", DRIVER_NAME,
		       pwm_freq);
	}
	previous_pwm_freq = pwm_freq;
	return ret;
}

/*
 *  pwm_motorl_write - Set value to pwm driver
 *  Write function of /dev/rtmotor_raw_l
 */
static ssize_t pwm_motorl_write(struct file *filep, const char __user *buf,
				size_t count, loff_t *pos)
{
	int ret = -1;
	int pwm_freq = 0;

	if (count < 0)
		return 0;

	ret = kstrtoint_from_user(buf, count, 10, &pwm_freq);
	if (ret) {
		printk(KERN_ERR "%s: error parsing string to int in %s()\n",
		       DRIVER_NAME, __func__);
		return ret;
	}

	ret = pwm_motorl_write_core(pwm_freq);
	if (ret < 0)
		return ret;
	return count;
}

static ssize_t pwm_motorl_write_core(int pwm_freq)
{
	// struct i2c_device_info *dev_info = filep->private_data;
	struct i2c_device_info *dev_info = i2c_pwm0_dev_info;

	int ret = 0;
	static int previous_pwm_freq = 0;

	if (pwm_freq < 0) {
		gpio_set_value(GPIO_MOTOR_DIR_L, 1);
		pwm_freq *= -1;
	} else {
		gpio_set_value(GPIO_MOTOR_DIR_L, 0);
	}

	if (pwm_freq != previous_pwm_freq) {
		// i2c_pwm_set_all(dev_info, 0, 0);
		if (pwm_freq) {
			ret = i2c_pwm_set_freq(dev_info, pwm_freq);
			if (ret) {
				printk(KERN_ERR
				       "%s: error i2c_pwm_set_freq in %s()\n",
				       DRIVER_NAME, __func__);
				return ret;
			}
			ret = i2c_pwm_set(dev_info, 0, 0, 2048);
			if (ret) {
				printk(KERN_ERR
				       "%s: error i2c_pwm_set in %s()\n",
				       DRIVER_NAME, __func__);
				return ret;
			}
		} else {
			ret = i2c_pwm_set(dev_info, 0, 0, 0);
			if (ret) {
				printk(KERN_ERR
				       "%s: error i2c_pwm_set in %s()\n",
				       DRIVER_NAME, __func__);
				return ret;
			}
		}
		printk(KERN_DEBUG "%s: set pwm driver freq %d\n", DRIVER_NAME,
		       pwm_freq);
	}
	previous_pwm_freq = pwm_freq;
	return ret;
}

/*
 * Device File Operation
 */

static int dev_open(struct inode *inode, struct file *filep)
{
	int *minor = (int *)kmalloc(sizeof(int), GFP_KERNEL);
	// int major = MAJOR(inode->i_rdev);
	*minor = MINOR(inode->i_rdev);
	// printk(KERN_INFO "open request major:%d minor: %d \n", major,
	// *minor);
	filep->private_data = (void *)minor;
	return 0;
}

static int dev_release(struct inode *inode, struct file *filep)
{
	kfree(filep->private_data);
	return 0;
}

static int i2c_dev_open(struct inode *inode, struct file *filep)
{
	struct i2c_device_info *dev_info;
	// int major = MAJOR(inode->i_rdev);
	// dev_info = container_of(major, struct i2c_device_info, device_major);
	dev_info = container_of(inode->i_cdev, struct i2c_device_info, cdev);
	if (dev_info == NULL || dev_info->client == NULL) {
		printk(KERN_ERR "%s: i2c dev_open failed.\n", DRIVER_NAME);
	}
	filep->private_data = dev_info;
	return 0;
}

static int i2c_dev_release(struct inode *inode, struct file *filep)
{
	/* no need to kfree filep->private_data */
	// kfree(filep->private_data);
	return 0;
}

/* --- Device File Operations --- */
/* /dev/rtled */
static struct file_operations led_fops = {
    .open = dev_open,
    .release = dev_release,
    .write = led_write,
};
/* /dev/rtswitch */
static struct file_operations sw_fops = {
    .open = dev_open,
    .read = sw_read,
    .release = dev_release,
};
/* /dev/rtswitches */
static struct file_operations sws_fops = {
    .open = dev_open,
    .read = sws_read,
    .release = dev_release,
};
/* /dev/rtlightsensor */
static struct file_operations sensor_fops = {
    .open = dev_open,
    .read = sensor_read,
    .release = dev_release,
};
/* /dev/rtmotoren */
static struct file_operations motoren_fops = {
    .open = dev_open,
    .write = motoren_write,
    .release = dev_release,
};
/* /dev/rtmotor */
static struct file_operations motor_fops = {
    .open = dev_open,
    .write = motor_write,
    .release = dev_release,
};
/* /dev/rtcounter_* */
static struct file_operations rtcnt_fops = {
    .open = i2c_dev_open,
    .release = i2c_dev_release,
    .read = rtcnt_read,
    .write = rtcnt_write,
};
/* /dev/rtbuzzer */
static struct file_operations buzzer_fops = {
    .open = dev_open,
    .release = dev_release,
    .write = pwm_buzzer_write,
};
/* /dev/rtmotor_raw_r */
static struct file_operations motorrawr_fops = {
    .open = dev_open,
    .release = dev_release,
    .write = pwm_motorr_write,
};
/* /dev/rtmotor_raw_l */
static struct file_operations motorrawl_fops = {
    .open = dev_open,
    .release = dev_release,
    .write = pwm_motorl_write,
};

/* --- Device Driver Registration and Device File Creation --- */
/* /dev/rtled0,/dev/rtled1,/dev/rtled2,/dev/rtled3 */
static int led_register_dev(void)
{
	int retval;
	dev_t dev;
	dev_t devno;
	int i;

	retval = alloc_chrdev_region(&dev, DEV_MINOR, NUM_DEV_LED, DEVNAME_LED);

	if (retval < 0) {
		printk(KERN_ERR "%s: alloc_chrdev_region failed.\n", __func__);
		return retval;
	}
	_major_led = MAJOR(dev);

	class_led = class_create(THIS_MODULE, DEVNAME_LED);
	if (IS_ERR(class_led)) {
		return PTR_ERR(class_led);
	}

	for (i = 0; i < NUM_DEV_LED; i++) {
		devno = MKDEV(_major_led, _minor_led + i);

		cdev_init(&(cdev_array[cdev_index]), &led_fops);
		cdev_array[cdev_index].owner = THIS_MODULE;
		if (cdev_add(&(cdev_array[cdev_index]), devno, 1) < 0) {
			printk(KERN_ERR "%s: cdev_add failed minor = %d\n",
			       __func__, _minor_led + i);
		} else {
			device_create(class_led, NULL, devno, NULL,
				      DEVNAME_LED "%u", _minor_led + i);
		}
		cdev_index++;
	}
	return 0;
}

/* /dev/rtswitch0, /dev/rtswitch1, /dev/rtswitch2 */
static int switch_register_dev(void)
{
	int retval;
	dev_t dev;
	int i;
	dev_t devno;

	retval = alloc_chrdev_region(&dev, DEV_MINOR, NUM_DEV_SWITCH,
				     DEVNAME_SWITCH);

	if (retval < 0) {
		printk(KERN_ERR "%s: alloc_chrdev_region failed.\n", __func__);
		return retval;
	}
	_major_switch = MAJOR(dev);

	class_switch = class_create(THIS_MODULE, DEVNAME_SWITCH);
	if (IS_ERR(class_switch)) {
		return PTR_ERR(class_switch);
	}

	for (i = 0; i < NUM_DEV_SWITCH; i++) {
		devno = MKDEV(_major_switch, _minor_switch + i);
		cdev_init(&(cdev_array[cdev_index]), &sw_fops);
		cdev_array[cdev_index].owner = THIS_MODULE;

		if (cdev_add(&(cdev_array[cdev_index]), devno, 1) < 0) {
			printk(KERN_ERR "%s: cdev_add failed minor = %d\n",
			       __func__, _minor_switch + i);
		} else {
			device_create(class_switch, NULL, devno, NULL,
				      DEVNAME_SWITCH "%u", _minor_switch + i);
		}
		cdev_index++;
	}
	return 0;
}

/* /dev/rtswitches0 */
static int switches_register_dev(void)
{
	int retval;
	dev_t dev;
	dev_t devno;

	retval = alloc_chrdev_region(&dev, DEV_MINOR, NUM_DEV_SWITCHES,
				     DEVNAME_SWITCHES);

	if (retval < 0) {
		printk(KERN_ERR "%s: alloc_chrdev_region failed.\n", __func__);
		return retval;
	}
	_major_switches = MAJOR(dev);

	class_switches = class_create(THIS_MODULE, DEVNAME_SWITCHES);
	if (IS_ERR(class_switches)) {
		return PTR_ERR(class_switches);
	}

	devno = MKDEV(_major_switches, _minor_switches);
	cdev_init(&(cdev_array[cdev_index]), &sws_fops);
	cdev_array[cdev_index].owner = THIS_MODULE;
	if (cdev_add(&(cdev_array[cdev_index]), devno, 1) < 0) {
		printk(KERN_ERR "%s: cdev_add failed minor = %d\n", __func__,
		       _minor_switches);
	} else {
		device_create(class_switches, NULL, devno, NULL,
			      DEVNAME_SWITCHES "%u", _minor_switches);
	}

	cdev_index++;

	return 0;
}

/* /dev/rtmotoren0 */
static int motoren_register_dev(void)
{
	int retval;
	dev_t dev;
	dev_t devno;

	retval = alloc_chrdev_region(&dev, DEV_MINOR, NUM_DEV_MOTOREN,
				     DEVNAME_MOTOREN);

	if (retval < 0) {
		printk(KERN_ERR "%s: alloc_chrdev_region failed.\n", __func__);
		return retval;
	}
	_major_motoren = MAJOR(dev);

	class_motoren = class_create(THIS_MODULE, DEVNAME_MOTOREN);
	if (IS_ERR(class_motoren)) {
		return PTR_ERR(class_motoren);
	}

	devno = MKDEV(_major_motoren, _minor_motoren);
	cdev_init(&(cdev_array[cdev_index]), &motoren_fops);
	cdev_array[cdev_index].owner = THIS_MODULE;
	if (cdev_add(&(cdev_array[cdev_index]), devno, 1) < 0) {
		printk(KERN_ERR "%s: cdev_add failed minor = %d\n", __func__,
		       _minor_motoren);
	} else {
		device_create(class_motoren, NULL, devno, NULL,
			      DEVNAME_MOTOREN "%u", _minor_motoren);
	}

	cdev_index++;

	return 0;
}

/* /dev/rtmotor0 */
static int motor_register_dev(void)
{
	int retval;
	dev_t dev;
	dev_t devno;

	/* 空いているメジャー番号を使ってメジャー
		&マイナー番号をカーネルに登録する */
	retval = alloc_chrdev_region(&dev, /* 結果を格納するdev_t構造体 */
				     DEV_MINOR, /* ベースマイナー番号 */
				     NUM_DEV_MOTOR, /* デバイスの数 */
				     DEVNAME_MOTOR /* デバイスドライバの名前 */
	);

	if (retval < 0) {
		printk(KERN_ERR "alloc_chrdev_region failed.\n");
		return retval;
	}
	_major_motor = MAJOR(dev);

	/* デバイスクラスを作成する */
	class_motor = class_create(THIS_MODULE, DEVNAME_MOTOR);
	if (IS_ERR(class_motor)) {
		return PTR_ERR(class_motor);
	}

	devno = MKDEV(_major_motor, _minor_motor);
	/* キャラクタデバイスとしてこのモジュールをカーネルに登録する */
	cdev_init(&(cdev_array[cdev_index]), &motor_fops);
	cdev_array[cdev_index].owner = THIS_MODULE;
	if (cdev_add(&(cdev_array[cdev_index]), devno, 1) < 0) {
		/* 登録に失敗した */
		printk(KERN_ERR "cdev_add failed minor = %d\n", _minor_motor);
	} else {
		/* デバイスノードの作成 */
		device_create(class_motor, NULL, devno, NULL,
			      DEVNAME_MOTOR "%u", _minor_motor);
	}

	cdev_index++;

	return 0;
}

/* /dev/rtlightsensor0 */
static int sensor_register_dev(void)
{
	int retval;
	dev_t dev;
	dev_t devno;

	retval = alloc_chrdev_region(&dev, DEV_MINOR, NUM_DEV_SENSOR,
				     DEVNAME_SENSOR);

	if (retval < 0) {
		printk(KERN_ERR "%s: alloc_chrdev_region failed.\n", __func__);
		return retval;
	}
	_major_sensor = MAJOR(dev);

	class_sensor = class_create(THIS_MODULE, DEVNAME_SENSOR);
	if (IS_ERR(class_sensor)) {
		return PTR_ERR(class_sensor);
	}

	devno = MKDEV(_major_sensor, _minor_sensor);
	cdev_init(&(cdev_array[cdev_index]), &sensor_fops);
	cdev_array[cdev_index].owner = THIS_MODULE;
	if (cdev_add(&(cdev_array[cdev_index]), devno, 1) < 0) {
		printk(KERN_ERR "%s: cdev_add failed minor = %d\n", __func__,
		       _minor_sensor);
	} else {
		device_create(class_sensor, NULL, devno, NULL,
			      DEVNAME_SENSOR "%u", _minor_sensor);
	}
	cdev_index++;
	return 0;
}

/*
 * mcp3204_remove - remove function lined with spi_dirver
 */
static int mcp3204_remove(struct spi_device *spi)
{
	struct mcp3204_drvdata *data;
	/* get drvdata */
	data = (struct mcp3204_drvdata *)spi_get_drvdata(spi);
	/* free kernel memory */
	kfree(data);
	// printk(KERN_INFO "%s: mcp3204 removed\n", DRIVER_NAME);
	return 0;
}

/*
 * mcp3204_probe - probe function lined with spi_dirver
 */
static int mcp3204_probe(struct spi_device *spi)
{
	struct mcp3204_drvdata *data;

	spi->max_speed_hz = mcp3204_info.max_speed_hz;
	spi->mode = mcp3204_info.mode;
	spi->bits_per_word = 8;

	if (spi_setup(spi)) {
		printk(KERN_ERR "%s:spi_setup failed!\n", __func__);
		return -ENODEV;
	}

	/* alloc kernel memory */
	data = kzalloc(sizeof(struct mcp3204_drvdata), GFP_KERNEL);
	if (data == NULL) {
		printk(KERN_ERR "%s:kzalloc() failed!\n", __func__);
		return -ENODEV;
	}

	data->spi = spi;

	mutex_init(&data->lock);

	// memset(data->tx, 0, MCP320X_PACKET_SIZE);
	// memset(data->rx, 0, MCP320X_PACKET_SIZE);

	data->xfer.tx_buf = data->tx;
	data->xfer.rx_buf = data->rx;
	data->xfer.bits_per_word = 8;
	data->xfer.len = MCP320X_PACKET_SIZE;
	data->xfer.cs_change = 0;
	data->xfer.delay_usecs = 0;
	data->xfer.speed_hz = 100000;

	spi_message_init_with_transfers(&data->msg, &data->xfer, 1);

	/* set drvdata */
	spi_set_drvdata(spi, data);

	// printk(KERN_INFO "%s: mcp3204 probed", DRIVER_NAME);

	return 0;
}

/*
 * mcp3204_get_value - get sensor data from MCP3204
 * called by 'sensor_read'
 */
static unsigned int mcp3204_get_value(int channel)
{
	struct device *dev;
	struct mcp3204_drvdata *data;
	struct spi_device *spi;
	char str[128];
	struct spi_master *master;

	unsigned int r = 0;
	unsigned char c = channel & 0x03;

	master = spi_busnum_to_master(mcp3204_info.bus_num);
	snprintf(str, sizeof(str), "%s.%u", dev_name(&master->dev),
		 mcp3204_info.chip_select);

	dev = bus_find_device_by_name(&spi_bus_type, NULL, str);
	spi = to_spi_device(dev);
	data = (struct mcp3204_drvdata *)spi_get_drvdata(spi);

	mutex_lock(&data->lock);
	data->tx[0] = 1 << 2;  // start bit
	data->tx[0] |= 1 << 1; // Single
	data->tx[1] = c << 6;  // channel
	data->tx[2] = 0;

	if (spi_sync(data->spi, &data->msg)) {
		printk(KERN_INFO "%s: spi_sync_transfer returned non zero\n",
		       __func__);
	}

	mutex_unlock(&data->lock);

	r = (data->rx[1] & 0xf) << 8;
	r |= data->rx[2];

	printk(KERN_DEBUG "%s: get result on ch[%d] : %04d\n", __func__,
	       channel, r);

	return r;
}

/*
 * spi_remove_device - remove SPI device
 * called by mcp3204_init and mcp3204_exit
 */
static void spi_remove_device(struct spi_master *master, unsigned int cs)
{
	struct device *dev;
	char str[128];

	snprintf(str, sizeof(str), "%s.%u", dev_name(&master->dev), cs);

	dev = bus_find_device_by_name(&spi_bus_type, NULL, str);
	if (dev) {
		device_del(dev);
	}
}

/*
 * mcp3204_init - initialize MCP3204
 * called by 'dev_init_module'
 */
static int mcp3204_init(void)
{
	struct spi_master *master;
	struct spi_device *spi_device;

	spi_register_driver(&mcp3204_driver);

	mcp3204_info.bus_num = spi_bus_num;
	mcp3204_info.chip_select = spi_chip_select;

	master = spi_busnum_to_master(mcp3204_info.bus_num);

	if (!master) {
		printk(KERN_ERR "%s: spi_busnum_to_master returned NULL\n",
		       __func__);
		spi_unregister_driver(&mcp3204_driver);
		return -ENODEV;
	}

	spi_remove_device(master, mcp3204_info.chip_select);

	spi_device = spi_new_device(master, &mcp3204_info);
	if (!spi_device) {
		printk(KERN_ERR "%s: spi_new_device returned NULL\n", __func__);
		spi_unregister_driver(&mcp3204_driver);
		return -ENODEV;
	}

	return 0;
}

/*
 * mcp3204_exit - cleanup MCP3204
 * called by dev_cleanup_module()
 */
static void mcp3204_exit(void)
{
	struct spi_master *master;

	master = spi_busnum_to_master(mcp3204_info.bus_num);

	if (master) {
		spi_remove_device(master, mcp3204_info.chip_select);
	} else {
		printk(KERN_ERR "mcp3204 remove error\n");
	}

	spi_unregister_driver(&mcp3204_driver);
}

/*
 * rtcntr_i2c_register_dev - I2C pulse counter
 * called from i2c_rtcnt_probe()
 */
static int rtcntr_i2c_register_dev(struct i2c_device_info *dev_info)
{
	int minor;
	int alloc_ret = 0;
	int cdev_err = 0;
	dev_t dev;

	/* 空いているメジャー番号を確保する */
	alloc_ret =
	    alloc_chrdev_region(&dev, DEV_MINOR, NUM_DEV_CNTR, DEVNAME_CNTR);
	if (alloc_ret != 0) {
		printk(KERN_ERR "alloc_chrdev_region = %d\n", alloc_ret);
		return -1;
	}

	/* 取得したdev( = メジャー番号 +  マイナー番号)
	 * からメジャー番号を取得して保持しておく */
	dev_info->device_major = MAJOR(dev);
	dev = MKDEV(dev_info->device_major, DEV_MINOR);
	printk(KERN_DEBUG "rtcntr_i2c major:%d, minor:%d\n",
	       dev_info->device_major, DEV_MINOR);

	/* cdev構造体の初期化とシステムコールハンドラテーブルの登録 */
	cdev_init(&dev_info->cdev, &rtcnt_fops);
	dev_info->cdev.owner = THIS_MODULE;

	/* このデバイスドライバ(cdev)をカーネルに登録する */
	cdev_err = cdev_add(&dev_info->cdev, dev, NUM_DEV_CNTR);
	if (cdev_err != 0) {
		printk(KERN_ERR "cdev_add = %d\n", alloc_ret);
		unregister_chrdev_region(dev, NUM_DEV_CNTR);
		return -1;
	}

	/* このデバイスのクラス登録をする(/sys/class/mydevice/ を作る) */
	dev_info->device_class = class_create(THIS_MODULE, DEVNAME_CNTR);
	if (IS_ERR(dev_info->device_class)) {
		printk(KERN_ERR "class_create\n");
		cdev_del(&dev_info->cdev);
		unregister_chrdev_region(dev, NUM_DEV_CNTR);
		return -1;
	}

	for (minor = DEV_MINOR; minor < DEV_MINOR + NUM_DEV_CNTR; minor++) {
		device_create(dev_info->device_class, NULL,
			      MKDEV(dev_info->device_major, minor), NULL,
			      "rtcounter_r%d", minor);
	}

	return 0;
}

/*
 * rtcntl_i2c_register_dev - I2C pulse counter
 * called from i2c_rtcnt_probe()
 */
static int rtcntl_i2c_register_dev(struct i2c_device_info *dev_info)
{
	int minor;
	int alloc_ret = 0;
	int cdev_err = 0;
	dev_t dev;

	/* 空いているメジャー番号を確保する */
	alloc_ret =
	    alloc_chrdev_region(&dev, DEV_MINOR, NUM_DEV_CNTL, DEVNAME_CNTL);
	if (alloc_ret != 0) {
		printk(KERN_ERR "alloc_chrdev_region = %d\n", alloc_ret);
		return -1;
	}

	/* 取得したdev( = メジャー番号 + マイナー番号)
	 * からメジャー番号を取得して保持しておく */
	dev_info->device_major = MAJOR(dev);
	dev = MKDEV(dev_info->device_major, DEV_MINOR);
	printk(KERN_DEBUG "rtcntl_i2c major:%d, minor:%d\n",
	       dev_info->device_major, DEV_MINOR);

	/* cdev構造体の初期化とシステムコールハンドラテーブルの登録 */
	cdev_init(&dev_info->cdev, &rtcnt_fops);
	dev_info->cdev.owner = THIS_MODULE;

	/* このデバイスドライバ(cdev)をカーネルに登録する */
	cdev_err = cdev_add(&dev_info->cdev, dev, NUM_DEV_CNTL);
	if (cdev_err != 0) {
		printk(KERN_ERR "cdev_add = %d\n", alloc_ret);
		unregister_chrdev_region(dev, NUM_DEV_CNTL);
		return -1;
	}

	/* このデバイスのクラス登録をする(/sys/class/mydevice/ を作る) */
	dev_info->device_class = class_create(THIS_MODULE, DEVNAME_CNTL);
	if (IS_ERR(dev_info->device_class)) {
		printk(KERN_ERR "class_create\n");
		cdev_del(&dev_info->cdev);
		unregister_chrdev_region(dev, NUM_DEV_CNTL);
		return -1;
	}

	/* /sys/class/mydevice/mydevice* を作る */
	for (minor = DEV_MINOR; minor < DEV_MINOR + NUM_DEV_CNTL; minor++) {
		device_create(dev_info->device_class, NULL,
			      MKDEV(dev_info->device_major, minor), NULL,
			      "rtcounter_l%d", minor);
	}

	return 0;
}

/*
 * i2c_rtcnt_probe - I2C pulse counter
 * called when I2C pulse counter probed
 */
static int i2c_rtcnt_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct i2c_device_info *dev_info;
	int msb = 0, lsb = 0;
	// printk(KERN_DEBUG "%s: probing i2c device", __func__);

	/* check i2c device */
	// printk(KERN_DEBUG "%s: checking i2c device", __func__);
	msb = i2c_smbus_read_byte_data(client, CNT_MSB_REG);
	lsb = i2c_smbus_read_byte_data(client, CNT_LSB_REG);
	if ((msb < 0) || (lsb < 0)) {
		printk(KERN_INFO
		       "%s: rtcounter not found, or wrong i2c device probed",
		       DRIVER_NAME);
		// printk(KERN_DEBUG "%s: addr 0x%x, msb %d, lsb %d", __func__,
		//        client->addr, msb, lsb);
		return -ENODEV;
	}
	printk(KERN_INFO "%s: new i2c device probed, id.name=%s, "
			 "id.driver_data=%d, addr=0x%x\n",
	       DRIVER_NAME, id->name, (int)(id->driver_data), client->addr);

	dev_info = (struct i2c_device_info *)devm_kzalloc(
	    &client->dev, sizeof(struct i2c_device_info), GFP_KERNEL);
	dev_info->client = client;
	i2c_set_clientdata(client, dev_info);
	mutex_init(&dev_info->lock);

	/* create character device */
	if ((int)(id->driver_data) == 0) {
		if (rtcntl_i2c_register_dev(dev_info))
			return -ENOMEM;
	} else if ((int)(id->driver_data) == 1) {
		if (rtcntr_i2c_register_dev(dev_info))
			return -ENOMEM;
	}

	return 0;
}

/*
 * i2c_counter_init - initialize I2C counter
 * called by init_mod()
 */
static int i2c_counter_init(void)
{
	int retval = 0;
	struct i2c_adapter *i2c_adap_l;
	struct i2c_adapter *i2c_adap_r;
	struct i2c_board_info i2c_board_info_l = {
	    I2C_BOARD_INFO(DEVNAME_CNTL, CNTL_I2C_ADDR)};
	struct i2c_board_info i2c_board_info_r = {
	    I2C_BOARD_INFO(DEVNAME_CNTR, CNTR_I2C_ADDR)};

	// printk(KERN_DEBUG "%s: initializing i2c device", __func__);
	retval = i2c_add_driver(&i2c_counter_driver);
	if (retval != 0) {
		printk(KERN_INFO "%s: failed adding i2c device", DRIVER_NAME);
		return retval;
	}

	/*
	 * 動的にデバイス実体を作成
	 * (https://www.kernel.org/doc/Documentation/i2c/instantiating-devices)
	 */
	// printk(KERN_DEBUG "%s: adding i2c device", __func__);
	i2c_adap_l = i2c_get_adapter(1);
	i2c_client_l = i2c_new_device(i2c_adap_l, &i2c_board_info_l);
	i2c_put_adapter(i2c_adap_l);
	// printk(KERN_DEBUG "%s: added i2c device rtcntl", __func__);

	// printk(KERN_DEBUG "%s: adding i2c device", __func__);
	i2c_adap_r = i2c_get_adapter(1);
	i2c_client_r = i2c_new_device(i2c_adap_r, &i2c_board_info_r);
	i2c_put_adapter(i2c_adap_r);
	// printk(KERN_DEBUG "%s: added i2c device rtcntr", __func__);

	return retval;
}

/*
 * i2c_counter_exit - cleanup I2C device
 * called by cleanup_mod()
 */
static void i2c_counter_exit(void)
{
	/* delete I2C driver */
	i2c_del_driver(&i2c_counter_driver);
	/* free memory */
	if (i2c_client_r)
		i2c_unregister_device(i2c_client_r);
	if (i2c_client_l)
		i2c_unregister_device(i2c_client_l);
}

/*
 * rtcnt_i2c_unregister_dev - I2C pulse counter
 * called from i2c_rtcnt_remove()
 */
static void rtcnt_i2c_unregister_dev(struct i2c_device_info *dev_info,
				     unsigned int devices)
{
	dev_t dev = MKDEV(dev_info->device_major, DEV_MINOR);
	int minor;
	printk(KERN_DEBUG "%s: deleting cdev for rtcnt", __func__);
	/* remove sysfs, /sys/class/rtcounter_[rl]/rtcounter_[rl]* */
	for (minor = DEV_MINOR; minor < DEV_MINOR + devices; minor++) {
		device_destroy(dev_info->device_class,
			       MKDEV(dev_info->device_major, minor));
	}
	/* remove rtcounter_[rl] class registration and delete
	 * /sys/class/rtcounter_[rl]/ */
	class_destroy(dev_info->device_class);
	/* remove cdev from the Linux kernel */
	cdev_del(&dev_info->cdev);
	/* remove major number registration */
	unregister_chrdev_region(dev, devices);
}

/*
 * i2c_counter_remove - I2C pulse counter
 * called when I2C pulse counter removed
 */
static int i2c_rtcnt_remove(struct i2c_client *client)
{
	struct i2c_device_info *dev_info;
	// printk(KERN_DEBUG "%s: removing i2c device 0x%x\n", __func__,
	// client->addr);
	dev_info = i2c_get_clientdata(client);
	if (client->addr == CNTL_I2C_ADDR)
		rtcnt_i2c_unregister_dev(dev_info, NUM_DEV_CNTL);
	if (client->addr == CNTR_I2C_ADDR)
		rtcnt_i2c_unregister_dev(dev_info, NUM_DEV_CNTR);
	printk(KERN_INFO "%s: i2c device 0x%x removed\n", DRIVER_NAME,
	       client->addr);
	return 0;
}

/*
 * buzzer_register_dev - register /dev/rtbuzzer0
 * called from i2c_pwm_probe()
 */
static int buzzer_register_dev(struct i2c_device_info *dev_info)
{
	int minor;
	int alloc_ret = 0;
	int cdev_err = 0;
	dev_t dev;

	printk(KERN_DEBUG "%s: adding cdev for pwm driver\n", __func__);

	/* 空いているメジャー番号を確保する */
	alloc_ret = alloc_chrdev_region(&dev, DEV_MINOR, NUM_DEV_BUZZER,
					DEVNAME_BUZZER);
	if (alloc_ret != 0) {
		printk(KERN_ERR "alloc_chrdev_region = %d\n", alloc_ret);
		return -1;
	}

	/* 取得したdev( = メジャー番号 + マイナー番号)
	 * からメジャー番号を取得して保持しておく */
	_major_buzzer = MAJOR(dev);
	dev = MKDEV(_major_buzzer, _minor_buzzer);
	// dev_info->device_major = MAJOR(dev);
	// dev = MKDEV(dev_info->device_major, DEV_MINOR);
	// printk(KERN_DEBUG "rtbuzzer_i2c major:%d, minor:%d\n",
	// dev_info->device_major, DEV_MINOR);

	/* cdev構造体の初期化とシステムコールハンドラテーブルの登録 */
	cdev_init(&(cdev_pwm_array[cdev_pwm_index]), &buzzer_fops);
	cdev_pwm_array[cdev_pwm_index].owner = THIS_MODULE;
	// cdev_init(&dev_info->cdev, &buzzer_fops);
	// dev_info->cdev.owner = THIS_MODULE;

	/* このデバイスドライバ(cdev)をカーネルに登録する */
	cdev_err =
	    cdev_add(&(cdev_pwm_array[cdev_pwm_index]), dev, NUM_DEV_BUZZER);
	// cdev_err = cdev_add(&dev_info->cdev, dev, NUM_DEV_BUZZER);
	if (cdev_err != 0) {
		printk(KERN_ERR "cdev_add = %d\n", cdev_err);
		unregister_chrdev_region(dev, NUM_DEV_BUZZER);
		return -1;
	}

	/* このデバイスのクラス登録をする(/sys/class/mydevice/ を作る) */
	class_buzzer = class_create(THIS_MODULE, DEVNAME_BUZZER);
	if (IS_ERR(class_buzzer)) {
		printk(KERN_ERR "class_create\n");
		cdev_del(&(cdev_pwm_array[cdev_pwm_index]));
		unregister_chrdev_region(dev, NUM_DEV_BUZZER);
		return -1;
	}
	// dev_info->device_class = class_create(THIS_MODULE, DEVNAME_BUZZER);
	// if (IS_ERR(dev_info->device_class)) {
	// 	printk(KERN_ERR "class_create\n");
	// 	cdev_del(&dev_info->cdev);
	// 	unregister_chrdev_region(dev, NUM_DEV_BUZZER);
	// 	return -1;
	// }

	/* /sys/class/mydevice/mydevice* を作る */
	for (minor = _minor_buzzer; minor < _minor_buzzer + NUM_DEV_BUZZER;
	     minor++) {
		device_create(class_buzzer, NULL, MKDEV(_major_buzzer, minor),
			      NULL, DEVNAME_BUZZER "%d", minor);
		// device_create(dev_info->device_class, NULL,
		// 	      MKDEV(dev_info->device_major, minor), NULL,
		// 	      DEVNAME_BUZZER "%d", minor);
	}

	cdev_pwm_index++;

	return 0;
}

/*
 * motorrawr_register_dev - register /dev/rtmotor_raw_r0
 * called from i2c_pwm_probe()
 */
static int motorrawr_register_dev(struct i2c_device_info *dev_info)
{
	int minor;
	int alloc_ret = 0;
	int cdev_err = 0;
	dev_t dev;

	printk(KERN_DEBUG "%s: adding cdev for pwm driver\n", __func__);

	/* 空いているメジャー番号を確保する */
	alloc_ret = alloc_chrdev_region(&dev, DEV_MINOR, NUM_DEV_MOTORRAWR,
					DEVNAME_MOTORRAWR);
	if (alloc_ret != 0) {
		printk(KERN_ERR "alloc_chrdev_region = %d\n", alloc_ret);
		return -1;
	}

	/* 取得したdev( = メジャー番号 + マイナー番号)
	 * からメジャー番号を取得して保持しておく */
	_major_motorrawr = MAJOR(dev);
	dev = MKDEV(_major_motorrawr, _minor_motorrawr);
	// dev_info->device_major = MAJOR(dev);
	// dev = MKDEV(dev_info->device_major, DEV_MINOR);
	// printk(KERN_DEBUG "rtmotorrawr_i2c major:%d, minor:%d\n",
	// dev_info->device_major, DEV_MINOR);

	/* cdev構造体の初期化とシステムコールハンドラテーブルの登録 */
	cdev_init(&(cdev_pwm_array[cdev_pwm_index]), &motorrawr_fops);
	cdev_pwm_array[cdev_pwm_index].owner = THIS_MODULE;
	// cdev_init(&dev_info->cdev, &motorrawr_fops);
	// dev_info->cdev.owner = THIS_MODULE;

	/* このデバイスドライバ(cdev)をカーネルに登録する */
	cdev_err =
	    cdev_add(&(cdev_pwm_array[cdev_pwm_index]), dev, NUM_DEV_MOTORRAWR);
	// cdev_err = cdev_add(&dev_info->cdev, dev, NUM_DEV_MOTORRAWR);
	if (cdev_err != 0) {
		printk(KERN_ERR "cdev_add = %d\n", cdev_err);
		unregister_chrdev_region(dev, NUM_DEV_MOTORRAWR);
		return -1;
	}

	/* このデバイスのクラス登録をする(/sys/class/mydevice/ を作る) */
	class_motorrawr = class_create(THIS_MODULE, DEVNAME_MOTORRAWR);
	if (IS_ERR(class_motorrawr)) {
		printk(KERN_ERR "class_create\n");
		cdev_del(&(cdev_pwm_array[cdev_pwm_index]));
		unregister_chrdev_region(dev, NUM_DEV_MOTORRAWR);
		return -1;
	}
	// dev_info->device_class = class_create(THIS_MODULE,
	// DEVNAME_MOTORRAWR); if (IS_ERR(dev_info->device_class)) {
	// 	printk(KERN_ERR "class_create\n");
	// 	cdev_del(&dev_info->cdev);
	// 	unregister_chrdev_region(dev, NUM_DEV_MOTORRAWR);
	// 	return -1;
	// }

	/* /sys/class/mydevice/mydevice* を作る */
	for (minor = _minor_motorrawr;
	     minor < _minor_motorrawr + NUM_DEV_MOTORRAWR; minor++) {
		device_create(class_motorrawr, NULL,
			      MKDEV(_major_motorrawr, minor), NULL,
			      DEVNAME_MOTORRAWR "%d", minor);
		// device_create(dev_info->device_class, NULL,
		// 	      MKDEV(dev_info->device_major, minor), NULL,
		// 	      DEVNAME_MOTORRAWR "%d", minor);
	}

	cdev_pwm_index++;

	return 0;
}

/*
 * motorrawl_register_dev - register /dev/rtmotor_raw_l0
 * called from i2c_pwm_probe()
 */
static int motorrawl_register_dev(struct i2c_device_info *dev_info)
{
	int minor;
	int alloc_ret = 0;
	int cdev_err = 0;
	dev_t dev;

	printk(KERN_DEBUG "%s: adding cdev for pwm driver\n", __func__);

	/* 空いているメジャー番号を確保する */
	alloc_ret = alloc_chrdev_region(&dev, DEV_MINOR, NUM_DEV_MOTORRAWL,
					DEVNAME_MOTORRAWL);
	if (alloc_ret != 0) {
		printk(KERN_ERR "alloc_chrdev_region = %d\n", alloc_ret);
		return -1;
	}

	/* 取得したdev( = メジャー番号 + マイナー番号)
	 * からメジャー番号を取得して保持しておく */
	_major_motorrawl = MAJOR(dev);
	dev = MKDEV(_major_motorrawl, _minor_motorrawl);
	// dev_info->device_major = MAJOR(dev);
	// dev = MKDEV(dev_info->device_major, DEV_MINOR);
	// printk(KERN_DEBUG "rtmotorrawl_i2c major:%d, minor:%d\n",
	// dev_info->device_major, DEV_MINOR);

	/* cdev構造体の初期化とシステムコールハンドラテーブルの登録 */
	cdev_init(&(cdev_pwm_array[cdev_pwm_index]), &motorrawl_fops);
	cdev_pwm_array[cdev_pwm_index].owner = THIS_MODULE;
	// cdev_init(&dev_info->cdev, &motorrawl_fops);
	// dev_info->cdev.owner = THIS_MODULE;

	/* このデバイスドライバ(cdev)をカーネルに登録する */
	cdev_err =
	    cdev_add(&(cdev_pwm_array[cdev_pwm_index]), dev, NUM_DEV_MOTORRAWL);
	// cdev_err = cdev_add(&dev_info->cdev, dev, NUM_DEV_MOTORRAWL);
	if (cdev_err != 0) {
		printk(KERN_ERR "cdev_add = %d\n", cdev_err);
		unregister_chrdev_region(dev, NUM_DEV_MOTORRAWL);
		return -1;
	}

	/* このデバイスのクラス登録をする(/sys/class/mydevice/ を作る) */
	class_motorrawl = class_create(THIS_MODULE, DEVNAME_MOTORRAWL);
	if (IS_ERR(class_motorrawl)) {
		printk(KERN_ERR "class_create\n");
		cdev_del(&(cdev_pwm_array[cdev_pwm_index]));
		unregister_chrdev_region(dev, NUM_DEV_MOTORRAWL);
		return -1;
	}
	// dev_info->device_class = class_create(THIS_MODULE,
	// DEVNAME_MOTORRAWL); if (IS_ERR(dev_info->device_class)) {
	// 	printk(KERN_ERR "class_create\n");
	// 	cdev_del(&dev_info->cdev);
	// 	unregister_chrdev_region(dev, NUM_DEV_MOTORRAWL);
	// 	return -1;
	// }

	/* /sys/class/mydevice/mydevice* を作る */
	for (minor = _minor_motorrawl;
	     minor < _minor_motorrawl + NUM_DEV_MOTORRAWL; minor++) {
		device_create(class_motorrawl, NULL,
			      MKDEV(_major_motorrawl, minor), NULL,
			      DEVNAME_MOTORRAWL "%d", minor);
		// device_create(dev_info->device_class, NULL,
		// 	      MKDEV(dev_info->device_major, minor), NULL,
		// 	      DEVNAME_MOTORRAWL "%d", minor);
	}

	cdev_pwm_index++;

	return 0;
}

/*
 * i2c_pwm_probe - I2C PWM driver
 * called when I2C PWM driver probed
 */
static int i2c_pwm_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	// struct i2c_device_info *dev_info;
	int mode1 = 0;
	int ret = -1;
	size_t size;
	// int msb = 0, lsb = 0;
	printk(KERN_DEBUG "%s: probing i2c device", __func__);

	printk(KERN_INFO "%s: new i2c device probed, id.name=%s, "
			 "id.driver_data=%d, addr=0x%x\n",
	       DRIVER_NAME, id->name, (int)(id->driver_data), client->addr);

	if ((int)(id->driver_data) == 0) {
		i2c_pwm0_dev_info = (struct i2c_device_info *)devm_kzalloc(
		    &client->dev, sizeof(struct i2c_device_info), GFP_KERNEL);
		i2c_pwm0_dev_info->client = client;
		i2c_set_clientdata(client, i2c_pwm0_dev_info);
		mutex_init(&i2c_pwm0_dev_info->lock);

		/* init pwm driver */
		ret = i2c_pwm_set_all(i2c_pwm0_dev_info, 0, 0);
		if (ret) {
			printk(KERN_INFO "%s: pwm driver not found, or wrong "
					 "i2c device probed",
			       DRIVER_NAME);
			mutex_unlock(&i2c_pwm0_dev_info->lock);
			return -ENODEV;
		}

		mutex_lock(&i2c_pwm0_dev_info->lock);

		ret = i2c_smbus_write_byte_data(client, PCA9685_MODE2,
						PCA9685_OUTDRV);
		if (ret) {
			printk(KERN_INFO "%s: pwm driver not found, or wrong "
					 "i2c device probed",
			       DRIVER_NAME);
			// printk(KERN_DEBUG "%s: addr 0x%x, msb %d, lsb %d",
			// __func__,
			//        client->addr, msb, lsb);
			mutex_unlock(&i2c_pwm0_dev_info->lock);
			return -ENODEV;
		}
		ret = i2c_smbus_write_byte_data(client, PCA9685_MODE1,
						PCA9685_ALLCALL);
		if (ret) {
			printk(KERN_INFO "%s: pwm driver not found, or wrong "
					 "i2c device probed",
			       DRIVER_NAME);
			// printk(KERN_DEBUG "%s: addr 0x%x, msb %d, lsb %d",
			// __func__,
			//        client->addr, msb, lsb);
			mutex_unlock(&i2c_pwm0_dev_info->lock);
			return -ENODEV;
		}
		udelay(500);
		/* soft reset pwm driver */
		mode1 = i2c_smbus_read_byte_data(client, PCA9685_MODE1);
		if (mode1 < 0) {
			printk(KERN_INFO "%s: pwm driver not found, or wrong "
					 "i2c device probed",
			       DRIVER_NAME);
			// printk(KERN_DEBUG "%s: addr 0x%x, msb %d, lsb %d",
			// __func__,
			//        client->addr, msb, lsb);
			return -ENODEV;
		}
		ret = i2c_smbus_write_byte_data(client, PCA9685_MODE1,
						mode1 & ~PCA9685_SLEEP);
		if (ret) {
			printk(KERN_INFO "%s: pwm driver not found, or wrong "
					 "i2c device probed",
			       DRIVER_NAME);
			// printk(KERN_DEBUG "%s: addr 0x%x, msb %d, lsb %d",
			// __func__,
			//        client->addr, msb, lsb);
			mutex_unlock(&i2c_pwm0_dev_info->lock);
			return -ENODEV;
		}
		udelay(500);
		mutex_unlock(&i2c_pwm0_dev_info->lock);
	} else if ((int)(id->driver_data) == 1) {
		i2c_pwm1_dev_info = (struct i2c_device_info *)devm_kzalloc(
		    &client->dev, sizeof(struct i2c_device_info), GFP_KERNEL);
		i2c_pwm1_dev_info->client = client;
		i2c_set_clientdata(client, i2c_pwm1_dev_info);
		mutex_init(&i2c_pwm1_dev_info->lock);

		/* init pwm driver */
		ret = i2c_pwm_set_all(i2c_pwm1_dev_info, 0, 0);
		if (ret) {
			printk(KERN_INFO "%s: pwm driver not found, or wrong "
					 "i2c device probed",
			       DRIVER_NAME);
			mutex_unlock(&i2c_pwm1_dev_info->lock);
			return -ENODEV;
		}

		mutex_lock(&i2c_pwm1_dev_info->lock);

		ret = i2c_smbus_write_byte_data(client, PCA9685_MODE2,
						PCA9685_OUTDRV);
		if (ret) {
			printk(KERN_INFO "%s: pwm driver not found, or wrong "
					 "i2c device probed",
			       DRIVER_NAME);
			// printk(KERN_DEBUG "%s: addr 0x%x, msb %d, lsb %d",
			// __func__,
			//        client->addr, msb, lsb);
			mutex_unlock(&i2c_pwm1_dev_info->lock);
			return -ENODEV;
		}
		ret = i2c_smbus_write_byte_data(client, PCA9685_MODE1,
						PCA9685_ALLCALL);
		if (ret) {
			printk(KERN_INFO "%s: pwm driver not found, or wrong "
					 "i2c device probed",
			       DRIVER_NAME);
			// printk(KERN_DEBUG "%s: addr 0x%x, msb %d, lsb %d",
			// __func__,
			//        client->addr, msb, lsb);
			mutex_unlock(&i2c_pwm1_dev_info->lock);
			return -ENODEV;
		}
		udelay(500);
		/* soft reset pwm driver */
		mode1 = i2c_smbus_read_byte_data(client, PCA9685_MODE1);
		if (mode1 < 0) {
			printk(KERN_INFO "%s: pwm driver not found, or wrong "
					 "i2c device probed",
			       DRIVER_NAME);
			// printk(KERN_DEBUG "%s: addr 0x%x, msb %d, lsb %d",
			// __func__,
			//        client->addr, msb, lsb);
			return -ENODEV;
		}
		ret = i2c_smbus_write_byte_data(client, PCA9685_MODE1,
						mode1 & ~PCA9685_SLEEP);
		if (ret) {
			printk(KERN_INFO "%s: pwm driver not found, or wrong "
					 "i2c device probed",
			       DRIVER_NAME);
			// printk(KERN_DEBUG "%s: addr 0x%x, msb %d, lsb %d",
			// __func__,
			//        client->addr, msb, lsb);
			mutex_unlock(&i2c_pwm1_dev_info->lock);
			return -ENODEV;
		}
		udelay(500);
		mutex_unlock(&i2c_pwm1_dev_info->lock);
	}

	/* create character device */
	size = sizeof(struct cdev) * NUM_DEV_PWM_TOTAL;
	cdev_pwm_array = (struct cdev *)kmalloc(size, GFP_KERNEL);
	if ((int)(id->driver_data) == 0) {
		printk(KERN_DEBUG "%s: going to add cdev", __func__);
		if (motorrawl_register_dev(i2c_pwm0_dev_info))
			return -ENOMEM;
	} else if ((int)(id->driver_data) == 1) {
		printk(KERN_DEBUG "%s: going to add cdev", __func__);
		if (buzzer_register_dev(i2c_pwm1_dev_info))
			return -ENOMEM;
		if (motorrawr_register_dev(i2c_pwm1_dev_info))
			return -ENOMEM;
	}

	return 0;
}

/*
 * i2c_pwm_init - initialize I2C pwm
 * called by init_mod()
 */
static int i2c_pwm_init(void)
{
	int retval = 0;
	struct i2c_adapter *i2c_adap_pwm0;
	struct i2c_adapter *i2c_adap_pwm1;
	struct i2c_board_info i2c_board_info_pwm0 = {
	    I2C_BOARD_INFO("pwmdriver0", PCA9685_L_I2C_ADDR)};
	struct i2c_board_info i2c_board_info_pwm1 = {
	    I2C_BOARD_INFO("pwmdriver1", PCA9685_R_I2C_ADDR)};

	printk(KERN_DEBUG "%s: initializing i2c device", __func__);
	retval = i2c_add_driver(&i2c_pwm_driver);
	if (retval != 0) {
		printk(KERN_INFO "%s: failed adding i2c device", DRIVER_NAME);
		return retval;
	}

	/*
	 * 動的にデバイス実体を作成
	 * (https://www.kernel.org/doc/Documentation/i2c/instantiating-devices)
	 */
	printk(KERN_DEBUG "%s: adding i2c pwm driver", __func__);
	i2c_adap_pwm0 = i2c_get_adapter(1);
	i2c_adap_pwm1 = i2c_get_adapter(1);
	i2c_client_pwm0 = i2c_new_device(i2c_adap_pwm0, &i2c_board_info_pwm0);
	i2c_client_pwm1 = i2c_new_device(i2c_adap_pwm1, &i2c_board_info_pwm1);
	i2c_put_adapter(i2c_adap_pwm0);
	i2c_put_adapter(i2c_adap_pwm1);
	printk(KERN_DEBUG "%s: added i2c pwm driver", __func__);

	return retval;
}

/*
 * i2c_pwm_exit - cleanup I2C device
 * called by cleanup_mod()
 */
static void i2c_pwm_exit(void)
{
	/* delete I2C driver */
	i2c_del_driver(&i2c_pwm_driver);
	/* free memory */
	if (i2c_client_pwm0)
		i2c_unregister_device(i2c_client_pwm0);
	if (i2c_client_pwm1)
		i2c_unregister_device(i2c_client_pwm1);
}

/*
 * i2c_pwm_remove - I2C PWM driver
 * called when I2C PWM driver removed
 */
static int i2c_pwm_remove(struct i2c_client *client)
{
	struct i2c_device_info *dev_info;
	int minor;
	dev_t dev;
	int i;
	// printk(KERN_DEBUG "%s: removing i2c device 0x%x\n", __func__,
	// client->addr);
	dev_info = i2c_get_clientdata(client);

	/* remove cdev from the Linux kernel */
	// printk(KERN_DEBUG "%s: deleting cdev for pwm driver", __func__);
	for (i = 0; i < NUM_DEV_PWM_TOTAL; i++) {
		cdev_del(&(cdev_pwm_array[i]));
	}

	/* /dev/rtbuzzer0 */
	dev = MKDEV(_major_buzzer, _minor_buzzer);
	printk(KERN_DEBUG "%s: deleting cdev for rtbuzzer", __func__);
	/* remove sysfs, /sys/class/<device name>/<device name>* */
	for (minor = _minor_buzzer; minor < _minor_buzzer + NUM_DEV_BUZZER;
	     minor++) {
		device_destroy(class_buzzer, MKDEV(_major_buzzer, minor));
	}
	/* remove rtbuzzer class registration and delete /sys/class/rtbuzzer/ */
	class_destroy(class_buzzer);
	/* remove major number registration */
	unregister_chrdev_region(dev, NUM_DEV_BUZZER);

	/* /dev/rtmotor_raw_r0 */
	dev = MKDEV(_major_motorrawr, _minor_motorrawr);
	printk(KERN_DEBUG "%s: deleting cdev for rtmotor_raw_r", __func__);
	/* remove sysfs, /sys/class/<device name>/<device name>* */
	for (minor = _minor_motorrawr;
	     minor < _minor_motorrawr + NUM_DEV_MOTORRAWR; minor++) {
		device_destroy(class_motorrawr, MKDEV(_major_motorrawr, minor));
	}
	/* remove rtmotor_raw_r class registration and delete
	 * /sys/class/rtmotor_raw_r/ */
	class_destroy(class_motorrawr);
	/* remove major number registration */
	unregister_chrdev_region(dev, NUM_DEV_MOTORRAWR);

	/* /dev/rtmotor_raw_l0 */
	dev = MKDEV(_major_motorrawl, _minor_motorrawl);
	printk(KERN_DEBUG "%s: deleting cdev for rtmotor_raw_l", __func__);
	/* remove sysfs, /sys/class/<device name>/<device name>* */
	for (minor = _minor_motorrawl;
	     minor < _minor_motorrawl + NUM_DEV_MOTORRAWL; minor++) {
		device_destroy(class_motorrawl, MKDEV(_major_motorrawl, minor));
	}
	/* remove rtmotor_raw_l class registration and delete
	 * /sys/class/rtmotor_raw_l/ */
	class_destroy(class_motorrawl);
	/* remove major number registration */
	unregister_chrdev_region(dev, NUM_DEV_MOTORRAWL);
	// rtcnt_i2c_unregister_dev(dev_info, NUM_DEV_BUZZER);
	// rtcnt_i2c_unregister_dev(dev_info, NUM_DEV_MOTORRAWR);
	// rtcnt_i2c_unregister_dev(dev_info, NUM_DEV_MOTORRAWL);

	printk(KERN_INFO "%s: i2c device 0x%x removed\n", DRIVER_NAME,
	       client->addr);

	kfree(cdev_pwm_array);
	return 0;
}

static int __init init_mod(void)
{
	int retval = 0;
	int registered_devices = 0;
	size_t size;

	printk(KERN_INFO "%s: loading %d devices...\n", DRIVER_NAME,
	       NUM_DEV_TOTAL);

	mutex_init(&lock);

	if (!gpio_is_valid(GPIO_SW0)) {
		printk(KERN_INFO "GPIO: invalid SW0 GPIO\n");
		return -ENODEV;
	}
	if (!gpio_is_valid(GPIO_SW1)) {
		printk(KERN_INFO "GPIO: invalid SW1 GPIO\n");
		return -ENODEV;
	}
	if (!gpio_is_valid(GPIO_SW2)) {
		printk(KERN_INFO "GPIO: invalid SW2 GPIO\n");
		return -ENODEV;
	}

	retval = gpio_request(GPIO_SW0, "sysfs");
	retval = gpio_request(GPIO_SW1, "sysfs");
	retval = gpio_request(GPIO_SW2, "sysfs");

	retval = gpio_direction_input(GPIO_SW0);
	retval = gpio_set_debounce(GPIO_SW0, DEBOUNCE_TIME);
	retval = gpio_export(GPIO_SW0, 0);

	retval = gpio_direction_input(GPIO_SW1);
	retval = gpio_set_debounce(GPIO_SW1, DEBOUNCE_TIME);
	retval = gpio_export(GPIO_SW1, 0);

	retval = gpio_direction_input(GPIO_SW2);
	retval = gpio_set_debounce(GPIO_SW2, DEBOUNCE_TIME);
	retval = gpio_export(GPIO_SW2, 0);

	if (!gpio_is_valid(GPIO_LED0)) {
		printk(KERN_INFO "GPIO: invalid LED0 GPIO\n");
		return -ENODEV;
	}
	if (!gpio_is_valid(GPIO_LED1)) {
		printk(KERN_INFO "GPIO: invalid LED1 GPIO\n");
		return -ENODEV;
	}
	if (!gpio_is_valid(GPIO_LED2)) {
		printk(KERN_INFO "GPIO: invalid LED2 GPIO\n");
		return -ENODEV;
	}
	if (!gpio_is_valid(GPIO_LED3)) {
		printk(KERN_INFO "GPIO: invalid LED3 GPIO\n");
		return -ENODEV;
	}

	retval = gpio_request(GPIO_LED0, "sysfs");
	retval = gpio_request(GPIO_LED1, "sysfs");
	retval = gpio_request(GPIO_LED2, "sysfs");
	retval = gpio_request(GPIO_LED3, "sysfs");

	retval = gpio_direction_output(GPIO_LED0, 0);
	retval = gpio_export(GPIO_LED0, 0);

	retval = gpio_direction_output(GPIO_LED1, 0);
	retval = gpio_export(GPIO_LED1, 0);

	retval = gpio_direction_output(GPIO_LED2, 0);
	retval = gpio_export(GPIO_LED2, 0);

	retval = gpio_direction_output(GPIO_LED3, 0);
	retval = gpio_export(GPIO_LED3, 0);

	if (!gpio_is_valid(GPIO_MOTOR_EN)) {
		printk(KERN_INFO "GPIO: invalid MOTOR EN GPIO\n");
		return -ENODEV;
	}
	retval = gpio_request(GPIO_MOTOR_EN, "sysfs");

	retval = gpio_direction_output(GPIO_MOTOR_EN, 0);
	retval = gpio_export(GPIO_MOTOR_EN, 1);

	if (!gpio_is_valid(GPIO_SEN_R)) {
		printk(KERN_INFO "GPIO: invalid SENSOR RIGHT GPIO\n");
		return -ENODEV;
	}
	if (!gpio_is_valid(GPIO_SEN_L)) {
		printk(KERN_INFO "GPIO: invalid SENSOR LEFT GPIO\n");
		return -ENODEV;
	}
	if (!gpio_is_valid(GPIO_SEN_RF)) {
		printk(KERN_INFO "GPIO: invalid SENSOR RIGHT FRONT GPIO\n");
		return -ENODEV;
	}
	if (!gpio_is_valid(GPIO_SEN_LF)) {
		printk(KERN_INFO "GPIO: invalid SENSOR LEFT FRONT GPIO\n");
		return -ENODEV;
	}

	retval = gpio_request(GPIO_SEN_R, "sysfs");
	retval = gpio_request(GPIO_SEN_L, "sysfs");
	retval = gpio_request(GPIO_SEN_RF, "sysfs");
	retval = gpio_request(GPIO_SEN_LF, "sysfs");
#ifdef ALTERNATIVE_SPI_CS
	retval = gpio_request(GPIO_SPI_CS, "sysfs");
#endif

	retval = gpio_direction_output(GPIO_SEN_R, 0);
	retval = gpio_export(GPIO_SEN_R, 0);
	retval = gpio_direction_output(GPIO_SEN_L, 0);
	retval = gpio_export(GPIO_SEN_L, 0);
	retval = gpio_direction_output(GPIO_SEN_RF, 0);
	retval = gpio_export(GPIO_SEN_RF, 0);
	retval = gpio_direction_output(GPIO_SEN_LF, 0);
	retval = gpio_export(GPIO_SEN_LF, 0);

#ifdef ALTERNATIVE_SPI_CS
	retval = gpio_direction_output(GPIO_SPI_CS, 1);
	retval = gpio_export(GPIO_SPI_CS, true);
#endif

	if (!gpio_is_valid(GPIO_MOTOR_DIR_R)) {
		printk(KERN_INFO "GPIO: invalid MOTORDIRR GPIO\n");
		return -ENODEV;
	}
	if (!gpio_is_valid(GPIO_MOTOR_DIR_L)) {
		printk(KERN_INFO "GPIO: invalid MOTORDIRL GPIO\n");
		return -ENODEV;
	}
	retval = gpio_direction_output(GPIO_MOTOR_DIR_R, 0);
	retval = gpio_export(GPIO_MOTOR_DIR_R, 0);
	retval = gpio_direction_output(GPIO_MOTOR_DIR_L, 0);
	retval = gpio_export(GPIO_MOTOR_DIR_L, 0);

	size = sizeof(struct cdev) * NUM_DEV_TOTAL;
	cdev_array = (struct cdev *)kmalloc(size, GFP_KERNEL);

	retval = led_register_dev();
	if (retval != 0) {
		printk(KERN_ALERT "%s: led driver register failed.\n",
		       DRIVER_NAME);
		return retval;
	}
	retval = switch_register_dev();
	if (retval != 0) {
		printk(KERN_ALERT "%s: switch driver register failed.\n",
		       DRIVER_NAME);
		return retval;
	}
	retval = switches_register_dev();
	if (retval != 0) {
		printk(KERN_ALERT "%s: switches driver register failed.\n",
		       DRIVER_NAME);
		return retval;
	}
	retval = motoren_register_dev();
	if (retval != 0) {
		printk(KERN_ALERT "%s: motoren driver register failed.\n",
		       DRIVER_NAME);
		return retval;
	}
	retval = motor_register_dev();
	if (retval != 0) {
		printk(KERN_ALERT "%s: motor driver register failed.\n",
		       DRIVER_NAME);
		return retval;
	}
	retval = sensor_register_dev();
	if (retval != 0) {
		printk(KERN_ALERT "%s: sensor driver register failed.\n",
		       DRIVER_NAME);
		return retval;
	}
	retval = mcp3204_init();
	if (retval != 0) {
		printk(KERN_ALERT
		       "%s: optical sensor driver register failed.\n",
		       DRIVER_NAME);
		return retval;
	}

	retval = i2c_counter_init();
	if (retval == 0) {
		registered_devices += NUM_DEV_RTCNT_TOTAL;
	} else {
		printk(KERN_ALERT
		       "%s: i2c counter device driver register failed.\n",
		       DRIVER_NAME);
		return retval;
	}

	retval = i2c_pwm_init();
	if (retval == 0) {
		registered_devices += NUM_DEV_PWM_TOTAL;
	} else {
		printk(KERN_ALERT
		       "%s: i2c pwm device driver register failed.\n",
		       DRIVER_NAME);
		return retval;
	}

	printk(KERN_INFO "%s: %d devices loaded.\n", DRIVER_NAME,
	       registered_devices + NUM_DEV_TOTAL);
	return 0;
}

static void __exit cleanup_mod(void)
{
	int i;
	dev_t devno;
	dev_t devno_top;

	printk(KERN_DEBUG "%s: removing %d cdev(s).\n", DRIVER_NAME,
	       NUM_DEV_TOTAL);
	for (i = 0; i < NUM_DEV_TOTAL; i++) {
		cdev_del(&(cdev_array[i]));
	}

	/* /dev/rtled0,1,2,3 */
	devno_top = MKDEV(_major_led, _minor_led);
	for (i = 0; i < NUM_DEV_LED; i++) {
		devno = MKDEV(_major_led, _minor_led + i);
		device_destroy(class_led, devno);
	}
	unregister_chrdev_region(devno_top, NUM_DEV_LED);

	/* /dev/rtswitch0,1,2 */
	devno_top = MKDEV(_major_switch, _minor_switch);
	for (i = 0; i < NUM_DEV_SWITCH; i++) {
		devno = MKDEV(_major_switch, _minor_switch + i);
		device_destroy(class_switch, devno);
	}
	unregister_chrdev_region(devno_top, NUM_DEV_SWITCH);

	/* /dev/rtswitches0 */
	devno = MKDEV(_major_switches, _minor_switches);
	device_destroy(class_switches, devno);
	unregister_chrdev_region(devno, NUM_DEV_SWITCH);

	/* /dev/rtmotoren0 */
	devno = MKDEV(_major_motoren, _minor_motoren);
	device_destroy(class_motoren, devno);
	unregister_chrdev_region(devno, NUM_DEV_MOTOREN);

	/* /dev/rtlightsensor0 */
	devno = MKDEV(_major_sensor, _minor_sensor);
	device_destroy(class_sensor, devno);
	unregister_chrdev_region(devno, NUM_DEV_SENSOR);

	/* /dev/rtmotor_raw_r0 */
	devno = MKDEV(_major_motorrawr, _minor_motorrawr);
	device_destroy(class_motorrawr, devno);
	unregister_chrdev_region(devno, NUM_DEV_MOTORRAWR);
	/* /dev/rtmotor_raw_l0 */
	devno = MKDEV(_major_motorrawl, _minor_motorrawl);
	device_destroy(class_motorrawl, devno);
	unregister_chrdev_region(devno, NUM_DEV_MOTORRAWL);
	/* /dev/rtmotor0 */
	devno = MKDEV(_major_motor, _minor_motor);
	device_destroy(class_motor, devno);
	unregister_chrdev_region(devno, NUM_DEV_MOTOR);

	class_destroy(class_led);
	class_destroy(class_switch);
	class_destroy(class_switches);
	class_destroy(class_sensor);
	class_destroy(class_motoren);

	/* remove MCP3204 */
	mcp3204_exit();

	/* remove I2C device */
	i2c_counter_exit();
	i2c_pwm_exit();

	kfree(cdev_array);

	mutex_destroy(&lock);

	/* GPIO unmap */
	/* set all gpio as low */
	gpio_set_value(GPIO_LED0, 0);
	gpio_set_value(GPIO_LED1, 0);
	gpio_set_value(GPIO_LED2, 0);
	gpio_set_value(GPIO_LED3, 0);
	gpio_set_value(GPIO_SEN_R, 0);
	gpio_set_value(GPIO_SEN_L, 0);
	gpio_set_value(GPIO_SEN_RF, 0);
	gpio_set_value(GPIO_SEN_LF, 0);
#ifdef ALTERNATIVE_SPI_CS
	gpio_set_value(GPIO_SPI_CS, 0);
#endif
	gpio_set_value(GPIO_MOTOR_EN, 0);
	/* sysfs: reverses the effect of exporting to userspace */
	gpio_unexport(GPIO_LED0);
	gpio_unexport(GPIO_LED1);
	gpio_unexport(GPIO_LED2);
	gpio_unexport(GPIO_LED3);
	gpio_unexport(GPIO_SEN_R);
	gpio_unexport(GPIO_SEN_L);
	gpio_unexport(GPIO_SEN_RF);
	gpio_unexport(GPIO_SEN_LF);
#ifdef ALTERNATIVE_SPI_CS
	gpio_unexport(GPIO_SPI_CS);
#endif
	gpio_unexport(GPIO_MOTOR_EN);
	gpio_unexport(GPIO_MOTOR_DIR_R);
	gpio_unexport(GPIO_MOTOR_DIR_L);
	/* reverse gpio_export() */
	gpio_free(GPIO_LED0);
	gpio_free(GPIO_LED1);
	gpio_free(GPIO_LED2);
	gpio_free(GPIO_LED3);
	gpio_free(GPIO_SW0);
	gpio_free(GPIO_SW1);
	gpio_free(GPIO_SW2);
	gpio_free(GPIO_SEN_R);
	gpio_free(GPIO_SEN_L);
	gpio_free(GPIO_SEN_RF);
	gpio_free(GPIO_SEN_LF);
#ifdef ALTERNATIVE_SPI_CS
	gpio_free(GPIO_SPI_CS);
#endif
	gpio_free(GPIO_MOTOR_EN);
	gpio_free(GPIO_MOTOR_DIR_R);
	gpio_free(GPIO_MOTOR_DIR_L);
	printk("module being removed at %lu\n", jiffies);
}

module_init(init_mod);
module_exit(cleanup_mod);
