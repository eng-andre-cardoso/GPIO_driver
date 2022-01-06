
#include <linux/types.h>

#define BCM2708_PERI_BASE       0x3f000000
#define GPIO_BASE (BCM2708_PERI_BASE + 0x200000) // GPIO controller


struct GpioRegisters
{
	uint32_t GPFSEL[6];
	uint32_t Reserved1;
	uint32_t GPSET[2];
	uint32_t Reserved2;
	uint32_t GPCLR[2];
};


/* Declaration of entry points */
static int raspi_gpio_open(struct inode* inode, struct file* filp);
static ssize_t raspi_gpio_read(struct file* filp, char* buf, size_t count, loff_t* f_pos);
static ssize_t raspi_gpio_write(struct file* filp, const char* buf, size_t count, loff_t* f_pos);
static int raspi_gpio_release(struct inode* inode, struct file* filp);





void SetGPIOFunction(struct GpioRegisters *s_pGpioRegisters, int GPIO, int functionCode);
void SetGPIOOutputValue(struct GpioRegisters *s_pGpioRegisters, int GPIO, bool outputValue);
