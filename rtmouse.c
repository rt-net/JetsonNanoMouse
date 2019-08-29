#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>

MODULE_AUTHOR("RT Corporation");
MODULE_DESCRIPTION("A simple driver for control Jetson Nano");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");

/* --- GPIO Pins --- */
#define gpioLED0 13       // PIN22
#define gpioLED1 15       // PIN18
#define gpioLED2 232      // PIN16
#define gpioLED3 79       // PIN16
#define gpioSW0 78	// PIN40
#define gpioSW1 12	// PIN37
#define gpioSW2 77	// PIN38
#define gpioSENR 194      // PIN15
#define gpioSENL 149      // PIN29
#define gpioSENRF 14      // PI13
#define gpioSENLF 50      // PIN11
#define gpioMOTOREN 200   // PIN31
#define gpioMOTORDIRR 168 // PIN32
#define gpioMOTORDIRL 216 // PIN7

#define MAX_BUFLEN 64
#define DEBOUNCE_TIME 50

#define DEV_MAJOR 0
#define DEV_MINOR 0

#define REG_GPIO_NAME "Jetson Nano GPIO"

#define NUM_DEV_LED 4
#define NUM_DEV_SWITCH 3
#define NUM_DEV_SENSOR 1
#define NUM_DEV_MOTOREN 1
#define NUM_DEV_TOTAL (NUM_DEV_LED + NUM_DEV_SWITCH + NUM_DEV_SENSOR + NUM_DEV_MOTOREN)

#define DRIVER_NAME "rtmouse"

#define DEVNAME_LED "rtled"
#define DEVNAME_SWITCH "rtswitch"
#define DEVNAME_SENSOR "rtlightsensor"
#define DEVNAME_MOTOREN "rtmotoren"

static struct cdev *cdev_array = NULL;
static struct class *class_led = NULL;
static struct class *class_switch = NULL;
static struct class *class_sensor = NULL;
static struct class *class_motoren = NULL;

static volatile int cdev_index = 0;
static volatile int open_counter = 0;

static struct mutex lock;

static int _major_led = DEV_MAJOR;
static int _minor_led = DEV_MINOR;

static int _major_switch = DEV_MAJOR;
static int _minor_switch = DEV_MINOR;

static int _major_sensor = DEV_MAJOR;
static int _minor_sensor = DEV_MINOR;

static int _major_motoren = DEV_MAJOR;
static int _minor_motoren = DEV_MINOR;

/* SPI Parameters */
static int spi_bus_num = 0;
static int spi_chip_select = 0;

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

/* --- Function Declarations --- */
static int mcp3204_remove(struct spi_device *spi);
static int mcp3204_probe(struct spi_device *spi);
static unsigned int mcp3204_get_value(int channel);

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

/* -- Device Addition -- */
MODULE_DEVICE_TABLE(spi, mcp3204_id);

/*
 * Turn On LEDs
 * return 0 : device close
 */
static int led_put(int ledno)
{
	switch (ledno) {
	case 0:
		gpio_set_value(gpioLED0, 1);
		break;
	case 1:
		gpio_set_value(gpioLED1, 1);
		break;
	case 2:
		gpio_set_value(gpioLED2, 1);
		break;
	case 3:
		gpio_set_value(gpioLED3, 1);
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
		gpio_set_value(gpioLED0, 0);
		break;
	case 1:
		gpio_set_value(gpioLED1, 0);
		break;
	case 2:
		gpio_set_value(gpioLED2, 0);
		break;
	case 3:
		gpio_set_value(gpioLED3, 0);
		break;
	}

	return 0;
}

/* --- Function for device file operations --- */

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
	unsigned int pin = gpioSW0;
	int minor = *((int *)filep->private_data);

	switch (minor) {
	case 0:
		pin = gpioSW0;
		break;
	case 1:
		pin = gpioSW1;
		break;
	case 2:
		pin = gpioSW2;
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

	/* get values through MCP3204 */
	/* Right side */
	or = mcp3204_get_value(R_AD_CH);
	gpio_set_value(gpioSENR, 1);
	udelay(usecs);
	r = mcp3204_get_value(R_AD_CH);
	gpio_set_value(gpioSENR, 0);
	udelay(usecs);
	/* Left side */
	ol = mcp3204_get_value(L_AD_CH);
	gpio_set_value(gpioSENL, 1);
	udelay(usecs);
	l = mcp3204_get_value(L_AD_CH);
	gpio_set_value(gpioSENL, 0);
	udelay(usecs);
	/* Right front side */
	orf = mcp3204_get_value(RF_AD_CH);
	gpio_set_value(gpioSENRF, 1);
	udelay(usecs);
	rf = mcp3204_get_value(RF_AD_CH);
	gpio_set_value(gpioSENRF, 0);
	udelay(usecs);
	/* Left front side */
	olf = mcp3204_get_value(LF_AD_CH);
	gpio_set_value(gpioSENLF, 1);
	udelay(usecs);
	lf = mcp3204_get_value(LF_AD_CH);
	gpio_set_value(gpioSENLF, 0);
	udelay(usecs);

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
			gpio_set_value(gpioMOTOREN, 1);
			break;
		case '0':
			gpio_set_value(gpioMOTOREN, 0);
			break;
		}
		return sizeof(char);
	}
	return 0;
}

static int dev_open(struct inode *inode, struct file *filep)
{
	int *minor = (int *)kmalloc(sizeof(int), GFP_KERNEL);
	// int major = MAJOR(inode->i_rdev);
	*minor = MINOR(inode->i_rdev);
	// printk(KERN_INFO "open request major:%d minor: %d \n", major,
	// *minor);
	filep->private_data = (void *)minor;
	open_counter++;
	return 0;
}

static int dev_release(struct inode *inode, struct file *filep)
{
	kfree(filep->private_data);
	open_counter--;
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

/* --- Device Driver Registration and Device File Creation --- */
/* /dev/rtled0,/dev/rtled1,/dev/rtled2 */
static int led_register_dev(void)
{
	int retval;
	dev_t dev;
	dev_t devno;
	int i;

	retval = alloc_chrdev_region(&dev, DEV_MINOR, NUM_DEV_LED, DEVNAME_LED);

	if (retval < 0) {
		printk(KERN_ERR "alloc_chrdev_region failed.\n");
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
			printk(KERN_ERR "cdev_add failed minor = %d\n",
			       _minor_led + i);
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
		printk(KERN_ERR "alloc_chrdev_region failed.\n");
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
			printk(KERN_ERR "cdev_add failed minor = %d\n",
			       _minor_switch + i);
		} else {
			device_create(class_switch, NULL, devno, NULL,
				      DEVNAME_SWITCH "%u", _minor_switch + i);
		}
		cdev_index++;
	}
	return 0;
}

/* /dev/rtmotoren0 */
static int motoren_register_dev(void)
{
	int retval;
	dev_t dev;
	dev_t devno;

	retval =
	    alloc_chrdev_region(&dev,
				DEV_MINOR,
				NUM_DEV_MOTOREN,
				DEVNAME_MOTOREN
				);

	if (retval < 0) {
		printk(KERN_ERR "alloc_chrdev_region failed.\n");
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
		printk(KERN_ERR "cdev_add failed minor = %d\n", _minor_motoren);
	} else {
		device_create(class_motoren, NULL, devno, NULL,
			      DEVNAME_MOTOREN "%u", _minor_motoren);
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
		printk(KERN_ERR "alloc_chrdev_region failed.\n");
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
		printk(KERN_ERR "cdev_add failed minor = %d\n", _minor_sensor);
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
	printk(KERN_INFO "%s: mcp3204 removed\n", DRIVER_NAME);
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

	printk(KERN_INFO "%s: mcp3204 probed", DRIVER_NAME);

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

	printk(KERN_INFO "%s: get result on ch[%d] : %04d\n", __func__, channel,
	       r);

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

static int __init init_mod(void)
{
	int retval = 0;
	size_t size;

	printk(KERN_INFO "loading %d devices...\n", NUM_DEV_TOTAL);

	mutex_init(&lock);

	if (!gpio_is_valid(gpioSW0)) {
		printk(KERN_INFO "GPIO: invalid SW0 GPIO\n");
		return -ENODEV;
	}
	if (!gpio_is_valid(gpioSW1)) {
		printk(KERN_INFO "GPIO: invalid SW1 GPIO\n");
		return -ENODEV;
	}
	if (!gpio_is_valid(gpioSW2)) {
		printk(KERN_INFO "GPIO: invalid SW2 GPIO\n");
		return -ENODEV;
	}

	retval = gpio_request(gpioSW0, "sysfs");
	retval = gpio_request(gpioSW1, "sysfs");
	retval = gpio_request(gpioSW2, "sysfs");

	retval = gpio_direction_input(gpioSW0);
	retval = gpio_set_debounce(gpioSW0, DEBOUNCE_TIME);
	retval = gpio_export(gpioSW0, 0);

	retval = gpio_direction_input(gpioSW1);
	retval = gpio_set_debounce(gpioSW1, DEBOUNCE_TIME);
	retval = gpio_export(gpioSW1, 0);

	retval = gpio_direction_input(gpioSW2);
	retval = gpio_set_debounce(gpioSW2, DEBOUNCE_TIME);
	retval = gpio_export(gpioSW2, 0);

	if (!gpio_is_valid(gpioLED0)) {
		printk(KERN_INFO "GPIO: invalid LED0 GPIO\n");
		return -ENODEV;
	}
	if (!gpio_is_valid(gpioLED1)) {
		printk(KERN_INFO "GPIO: invalid LED1 GPIO\n");
		return -ENODEV;
	}
	if (!gpio_is_valid(gpioLED2)) {
		printk(KERN_INFO "GPIO: invalid LED2 GPIO\n");
		return -ENODEV;
	}
	if (!gpio_is_valid(gpioLED3)) {
		printk(KERN_INFO "GPIO: invalid LED3 GPIO\n");
		return -ENODEV;
	}

	retval = gpio_request(gpioLED0, "sysfs");
	retval = gpio_request(gpioLED1, "sysfs");
	retval = gpio_request(gpioLED2, "sysfs");
	retval = gpio_request(gpioLED3, "sysfs");

	retval = gpio_direction_output(gpioLED0, 0);
	retval = gpio_export(gpioLED0, 0);

	retval = gpio_direction_output(gpioLED1, 0);
	retval = gpio_export(gpioLED1, 0);

	retval = gpio_direction_output(gpioLED2, 0);
	retval = gpio_export(gpioLED2, 0);

	retval = gpio_direction_output(gpioLED3, 0);
	retval = gpio_export(gpioLED3, 0);

	if (!gpio_is_valid(gpioMOTOREN)) {
		printk(KERN_INFO "GPIO: invalid MOTOR EN GPIO\n");
		return -ENODEV;
	}
	retval = gpio_request(gpioMOTOREN, "sysfs");

	retval = gpio_direction_output(gpioMOTOREN, 0);
	retval = gpio_export(gpioMOTOREN, 1);

	if (!gpio_is_valid(gpioSENR)) {
		printk(KERN_INFO "GPIO: invalid SENSOR RIGHT GPIO\n");
		return -ENODEV;
	}
	if (!gpio_is_valid(gpioSENL)) {
		printk(KERN_INFO "GPIO: invalid SENSOR LEFT GPIO\n");
		return -ENODEV;
	}
	if (!gpio_is_valid(gpioSENRF)) {
		printk(KERN_INFO "GPIO: invalid SENSOR RIGHT FRONT GPIO\n");
		return -ENODEV;
	}
	if (!gpio_is_valid(gpioSENLF)) {
		printk(KERN_INFO "GPIO: invalid SENSOR LEFT FRONT GPIO\n");
		return -ENODEV;
	}

	retval = gpio_request(gpioSENR, "sysfs");
	retval = gpio_request(gpioSENL, "sysfs");
	retval = gpio_request(gpioSENRF, "sysfs");
	retval = gpio_request(gpioSENLF, "sysfs");

	retval = gpio_direction_output(gpioSENR, 0);
	retval = gpio_export(gpioSENR, 0);
	retval = gpio_direction_output(gpioSENL, 0);
	retval = gpio_export(gpioSENL, 0);
	retval = gpio_direction_output(gpioSENRF, 0);
	retval = gpio_export(gpioSENRF, 0);
	retval = gpio_direction_output(gpioSENLF, 0);
	retval = gpio_export(gpioSENLF, 0);

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
	retval = motoren_register_dev();
	if (retval != 0) {
		printk(KERN_ALERT "%s: motoren driver register failed.\n",
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
	return 0;
}

static void __exit cleanup_mod(void)
{
	int i;
	dev_t devno;
	dev_t devno_top;

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

	/* /dev/rtmotoren0 */
	devno = MKDEV(_major_motoren, _minor_motoren);
	device_destroy(class_motoren, devno);
	unregister_chrdev_region(devno, NUM_DEV_MOTOREN);

	/* /dev/rtlightsensor0 */
	devno = MKDEV(_major_sensor, _minor_sensor);
	device_destroy(class_sensor, devno);
	unregister_chrdev_region(devno, NUM_DEV_SENSOR);

	class_destroy(class_led);
	class_destroy(class_switch);
	class_destroy(class_sensor);
	class_destroy(class_motoren);

	mcp3204_exit();

	kfree(cdev_array);

	mutex_destroy(&lock);

	/* GPIO unmap */
	/* set all gpio as low */
	gpio_set_value(gpioLED0, 0);
	gpio_set_value(gpioLED1, 0);
	gpio_set_value(gpioLED2, 0);
	gpio_set_value(gpioLED3, 0);
	gpio_set_value(gpioSENR, 0);
	gpio_set_value(gpioSENL, 0);
	gpio_set_value(gpioSENRF, 0);
	gpio_set_value(gpioSENLF, 0);
	gpio_set_value(gpioMOTOREN, 0);
	/* sysfs: reverses the effect of exporting to userspace */
	gpio_unexport(gpioLED0);
	gpio_unexport(gpioLED1);
	gpio_unexport(gpioLED2);
	gpio_unexport(gpioLED3);
	gpio_unexport(gpioSENR);
	gpio_unexport(gpioSENL);
	gpio_unexport(gpioSENRF);
	gpio_unexport(gpioSENLF);
	gpio_unexport(gpioMOTOREN);
	/* reverse gpio_export() */
	gpio_free(gpioLED0);
	gpio_free(gpioLED1);
	gpio_free(gpioLED2);
	gpio_free(gpioLED3);
	gpio_free(gpioSW0);
	gpio_free(gpioSW1);
	gpio_free(gpioSW2);
	gpio_free(gpioSENR);
	gpio_free(gpioSENL);
	gpio_free(gpioSENRF);
	gpio_free(gpioSENLF);
	gpio_free(gpioMOTOREN);
	printk("module being removed at %lu\n", jiffies);
}

module_init(init_mod);
module_exit(cleanup_mod);
