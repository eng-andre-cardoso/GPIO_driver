#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/module.h>
#include <asm/io.h>
#include <linux/timer.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/mm.h>

#include <linux/io.h>
//#include <mach/platform.h>

#include "utils.h"

/* User-defined macros */
#define NUM_GPIO_PINS 40
#define MAX_GPIO_NUMBER 26
#define DEVICE_NAME "raspi-gpio"
#define BUF_SIZE 512
#define INTERRUPT_DEVICE_NAME "gpio interrupt"



MODULE_LICENSE("GPL");

/* Device variables */
static struct class* ledDevice_class = NULL;
static struct device* ledDevice_device = NULL;
static dev_t ledDevice_majorminor;
static struct cdev c_dev;  // Character device structure


static struct class *s_pDeviceClass;
static struct device *s_pDeviceObject;
struct GpioRegisters *s_pGpioRegisters;



static struct file_operations raspi_gpio_fops = {
	.owner = THIS_MODULE,
	.open = raspi_gpio_open,
	.release = raspi_gpio_release,
	.read = raspi_gpio_read,
	.write = raspi_gpio_write,
};


/* Forward declaration of functions */
static int raspi_gpio_init(void);
static void raspi_gpio_exit(void);
unsigned int millis(void);
static irqreturn_t irq_handler(int irq, void* arg);
/* Global varibles for GPIO driver */
struct raspi_gpio_dev* raspi_gpio_devp[NUM_GPIO_PINS];
static dev_t first;
static struct class* raspi_gpio_class;
static unsigned int last_interrupt_time = 0;
static uint64_t epochMilli;


ssize_t raspi_gpio_write(struct file *pfile, const char __user *pbuff, size_t len, loff_t *off) { //tirar o static
	struct GpioRegisters *pdev; 
	
	pr_alert("%s: called (%u)\n",__FUNCTION__,len);
	if(unlikely(pfile->private_data == NULL))
		return -EFAULT;

	pdev = (struct GpioRegisters *)pfile->private_data;
	if (pbuff[0]=='0')
		SetGPIOOutputValue(pdev, LedGpioPin, 0);
	else
		SetGPIOOutputValue(pdev, LedGpioPin, 1);
	return len;
}

ssize_t raspi_gpio_read(struct file *pfile, char __user *p_buff,size_t len, loff_t *poffset){
	pr_alert("%s: called (%u)\n",__FUNCTION__,len);
	return 0;
}


int raspi_gpio_close(struct inode *p_inode, struct file * pfile){
	
	pr_alert("%s: called\n",__FUNCTION__);
	pfile->private_data = NULL;
	return 0;
}


int raspi_gpio_open(struct inode* p_indode, struct file *p_file){

	pr_alert("%s: called\n",__FUNCTION__);
	p_file->private_data = (struct GpioRegisters *) s_pGpioRegisters;
	return 0;
	
}


static int __init raspi_gpio__init(void) {
	int ret;
	struct device *dev_ret;
	int index = 0;

	s_pGpioRegisters = (struct GpioRegisters*)ioremap_nocache(GPIO_BASE, sizeof(struct GpioRegisters));
	pr_alert("map to virtual adresse: 0x%x\n", (unsigned)s_pGpioRegisters);
	return 0;

	pr_alert("%s: called\n",__FUNCTION__);

	if ((ret = alloc_chrdev_region(&first, 0, NUM_GPIO_PINS, DEVICE_NAME) < 0){
		printk(KERN_DEBUG "Cannot register device\n");
		return ret;
	}

	if (IS_ERR(raspi_gpio_class = class_create(THIS_MODULE, DEVICE_NAME))) {
		unregister_chrdev_region(first, NUM_GPIO_PINS);
		printk(KERN_DEBUG "Cannot create class %s\n", DEVICE_NAME);
		return PTR_ERR(raspi_gpio_class);
	}
	//substituido
	//if (IS_ERR(dev_ret = device_create(ledDevice_class, NULL, ledDevice_majorminor, NULL, DEVICE_NAME))) {
	//	class_destroy(ledDevice_class);
	//	unregister_chrdev_region(ledDevice_majorminor, 1);
	//	return PTR_ERR(dev_ret);
	//}

	//cdev_init(&c_dev, &ledDevice_fops);
	//c_dev.owner = THIS_MODULE;
	//if ((ret = cdev_add(&c_dev, ledDevice_majorminor, 1)) < 0) {
	//	printk(KERN_NOTICE "Error %d adding device", ret);
	//	device_destroy(ledDevice_class, ledDevice_majorminor);
	//	class_destroy(ledDevice_class);
	//	unregister_chrdev_region(ledDevice_majorminor, 1);
	//	return ret;
	//}

	// novo para varios devices

	for (i = 0; i < MAX_GPIO_NUMBER; i++) {
		if (i != 0 && i != 1 && i != 2 && i != 4 &&
			i != 6 && i != 9 && i != 14 && i != 17 &&
			i != 20 && i != 25 && i != 30 && i != 34 && i != 39) {
			raspi_gpio_devp[index] = kmalloc(sizeof(struct raspi_gpio_dev), GFP_KERNEL);
			if (!raspi_gpio_devp[index]) {
				printk("Bad kmalloc\n");
				return -ENOMEM;
			}
			/*if (gpio_request_one(i, GPIOF_OUT_INIT_LOW, NULL) < 0) {
				printk(KERN_ALERT "Error requesting GPIO %d\n", i);
				return -ENODEV;
			}*/
			SetGPIOFunction(s_pGpioRegisters, i, 0b001);//as output
			SetGPIOOutputValue(pdev, i, 0);//set to low value
			raspi_gpio_devp[index]->dir = out;
			raspi_gpio_devp[index]->state = low;
			raspi_gpio_devp[index]->irq_perm = false;
			raspi_gpio_devp[index]->irq_flag = IRQF_TRIGGER_RISING;
			raspi_gpio_devp[index]->irq_counter = 0;
			raspi_gpio_devp[index]->cdev.owner = THIS_MODULE;
			spin_lock_init(&raspi_gpio_devp[index]->lock);
			cdev_init(&raspi_gpio_devp[index]->cdev, &raspi_gpio_fops);
			if ((ret = cdev_add(&raspi_gpio_devp[index]->cdev, (first + i), 1))) {
				printk(KERN_ALERT "Error %d adding cdev\n", ret);
				
				for (i = 0; i < MAX_GPIO_NUMBER; i++) {
					if (i != 0 && i != 1 && i != 2 && i != 4 &&
						i != 6 && i != 9 && i != 14 && i != 17 &&
						i != 20 && i != 25 && i != 30 && i != 34 && i != 39) {
						device_destroy(raspi_gpio_class,MKDEV(MAJOR(first), MINOR(first) + i));
					}
				}
				class_destroy(raspi_gpio_class);
				unregister_chrdev_region(first, NUM_GPIO_PINS);
				return ret;
			}
			if (device_create(raspi_gpio_class, NULL, MKDEV(MAJOR(first), MINOR(first) + i), NULL, "raspiGpio%d", i) == NULL) {
				class_destroy(raspi_gpio_class);
				unregister_chrdev_region(first, NUM_GPIO_PINS);
				return -1;
			}
			index++;
		}
	}
	
}

static void __exit raspi_gpio__exit(void) {
	
	pr_alert("%s: called\n",__FUNCTION__);
	
	unregister_chrdev_region(first, NUM_GPIO_PINS);
	
	for (i = 0; i < NUM_GPIO_PINS; i++)
	kfree(raspi_gpio_devp[i]);

	for (i = 0; i < MAX_GPIO_NUMBER; i++) {
		if (i != 0 && i != 1 && i != 2 && i != 4 &&
			i != 6 && i != 9 && i != 14 && i != 17 &&
			i != 20 && i != 25 && i != 30 && i != 34 && i != 39) {
			SetGPIOFunction(s_pGpioRegisters, i, 0);
			device_destroy(raspi_gpio_class, MKDEV(MAJOR(first), MINOR(first) + i));
		}
	}
	iounmap(s_pGpioRegisters);
	cdev_del(&c_dev);
	class_destroy(raspio_gpio_class);
	
}

module_init(raspi_gpio__init);
module_exit(raspi_gpio__exit);
