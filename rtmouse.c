#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

MODULE_AUTHOR("RT Corporation");
MODULE_DESCRIPTION("A simple driver for controlling Jetson Nano");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");

static unsigned int gpioLED0 = 13;  // PIN22
static unsigned int gpioLED1 = 15;  // PIN18
static unsigned int gpioLED2 = 232; // PIN16
static unsigned int gpioLED3 = 79;  // PIN16
static unsigned int gpioSW0 = 77;   // PIN38
static unsigned int gpioSW1 = 12;   // PIN37
static unsigned int gpioSW2 = 78;   // PIN40
static unsigned int gpioSENR = 194;   // PIN15
static unsigned int gpioSENL = 216;   // PIN7
static unsigned int gpioSENRF = 14;   // PI13
static unsigned int gpioSENLF = 50;   // PIN11

static unsigned int MAX_BUFLEN = 64;
static unsigned int DEBOUNCE_TIME = 50;

static struct cdev *cdev_array = NULL;
static struct class *class_led = NULL;
static struct class *class_switch = NULL;

static volatile int cdev_index = 0;
static volatile int open_counter = 0;

#define DEV_MAJOR 0
#define DEV_MINOR 0

#define REG_GPIO_NAME "Jetson Nano GPIO"

#define NUM_DEV_LED 4
#define NUM_DEV_SWITCH 3
#define NUM_DEV_TOTAL (NUM_DEV_LED + NUM_DEV_SWITCH)

#define DEVNAME_LED "rtled"
#define DEVNAME_SWITCH "rtswitch"

static int _major_led = DEV_MAJOR;
static int _minor_led = DEV_MINOR;

static int _major_switch = DEV_MAJOR;
static int _minor_switch = DEV_MINOR;

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

	ret = gpio_get_value(pin);
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

static int __init init_mod(void)
{
	int retval = 0;
	size_t size;

	printk(KERN_INFO "loading %d devices...\n", NUM_DEV_TOTAL);

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
	retval = gpio_export(gpioSW0, false);

	retval = gpio_direction_input(gpioSW1);
	retval = gpio_set_debounce(gpioSW1, DEBOUNCE_TIME);
	retval = gpio_export(gpioSW1, false);

	retval = gpio_direction_input(gpioSW2);
	retval = gpio_set_debounce(gpioSW2, DEBOUNCE_TIME);
	retval = gpio_export(gpioSW2, false);

	if (!gpio_is_valid(gpioLED0)) {
		printk(KERN_INFO "GPIO: invalid LED0 GPIO\n");
		return -ENODEV;
	}
	if (!gpio_is_valid(gpioLED1)) {
		printk(KERN_INFO "GPIO: invalid LED0 GPIO\n");
		return -ENODEV;
	}
	if (!gpio_is_valid(gpioLED2)) {
		printk(KERN_INFO "GPIO: invalid LED0 GPIO\n");
		return -ENODEV;
	}
	if (!gpio_is_valid(gpioLED3)) {
		printk(KERN_INFO "GPIO: invalid LED0 GPIO\n");
		return -ENODEV;
	}

	retval = gpio_request(gpioLED0, "sysfs");
	retval = gpio_request(gpioLED1, "sysfs");
	retval = gpio_request(gpioLED2, "sysfs");
	retval = gpio_request(gpioLED3, "sysfs");

	retval = gpio_direction_output(gpioLED0, 0);
	retval = gpio_export(gpioLED0, false);

	retval = gpio_direction_output(gpioLED1, 0);
	retval = gpio_export(gpioLED1, false);

	retval = gpio_direction_output(gpioLED2, 0);
	retval = gpio_export(gpioLED2, false);

	retval = gpio_direction_output(gpioLED3, 0);
	retval = gpio_export(gpioLED3, false);

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
	retval = gpio_export(gpioSENR, false);
	retval = gpio_direction_output(gpioSENL, 0);
	retval = gpio_export(gpioSENL, false);
	retval = gpio_direction_output(gpioSENRF, 0);
	retval = gpio_export(gpioSENRF, false);
	retval = gpio_direction_output(gpioSENLF, 0);
	retval = gpio_export(gpioSENLF, false);

	size = sizeof(struct cdev) * NUM_DEV_TOTAL;
	cdev_array = (struct cdev *)kmalloc(size, GFP_KERNEL);

	retval = led_register_dev();
	if (retval != 0) {
		printk(KERN_ALERT " switch driver register failed.\n");
		return retval;
	}
	retval = switch_register_dev();
	if (retval != 0) {
		printk(KERN_ALERT " switch driver register failed.\n");
		return retval;
	}
	return 0;
}

static void __exit cleanup_mod(void)
{
	int i;
	dev_t devno;
	dev_t devno_top;

	gpio_set_value(gpioLED0, 0);
	gpio_set_value(gpioLED1, 0);
	gpio_set_value(gpioLED2, 0);
	gpio_set_value(gpioLED3, 0);
	gpio_unexport(gpioLED0);
	gpio_unexport(gpioLED1);
	gpio_unexport(gpioLED2);
	gpio_unexport(gpioLED3);
	gpio_free(gpioLED0);
	gpio_free(gpioLED1);
	gpio_free(gpioLED2);
	gpio_free(gpioLED3);
	gpio_free(gpioSW0);
	gpio_free(gpioSW1);
	gpio_free(gpioSW2);

	for (i = 0; i < NUM_DEV_TOTAL; i++) {
		cdev_del(&(cdev_array[i]));
	}

	devno_top = MKDEV(_major_led, _minor_led);
	for (i = 0; i < NUM_DEV_LED; i++) {
		devno = MKDEV(_major_led, _minor_led + i);
		device_destroy(class_led, devno);
	}
	unregister_chrdev_region(devno_top, NUM_DEV_LED);

	devno_top = MKDEV(_major_switch, _minor_switch);
	for (i = 0; i < NUM_DEV_SWITCH; i++) {
		devno = MKDEV(_major_switch, _minor_switch + i);
		device_destroy(class_switch, devno);
	}
	unregister_chrdev_region(devno_top, NUM_DEV_SWITCH);

	class_destroy(class_led);
	class_destroy(class_switch);

	kfree(cdev_array);
	printk("module being removed at %lu\n", jiffies);
}

module_init(init_mod);
module_exit(cleanup_mod);
