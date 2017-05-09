#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/fcntl.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/gpio.h>      // for 'gpio_to_irq()'

#include <linux/time.h>
#include <linux/timer.h>

#include <linux/interrupt.h>

#define INT_DEV_NAME    "intdev"
#define INT_DEV_MAJOR   240

// We will use beaglebone P9 hearder 12pin(GPIO1 28) as output,
// and P9 header 23pin as input (GPIO1 17)
//
// Beagle bone GPIO1 register starts from 0x4804C000 (Refer to data sheet).
// OE register sets GPIO pin as input(default 1)/output(0) by setting bit of pin number(0~31).
// DATAOUT register sets value of GPIO pin by setting bit of pin number(0~31).
// DATAIN register gets value of GPIO pin by getting bit of pin number(0~31).
#define GPIO1_ADDR            0x4804C000
#define OE_REG_OFFSET         0x134
#define DATAOUT_REG_OFFSET    0x13C
#define DATAIN_REG_OFFSET     0x138

#define OE_REG_ADDR           (GPIO1_ADDR+OE_REG_OFFSET)
#define DATAOUT_REG_ADDR      (GPIO1_ADDR+DATAOUT_REG_OFFSET)
#define DATAIN_REG_ADDR       (GPIO1_ADDR+DATAIN_REG_OFFSET)

#define P9_12_GPIO1_28        28
#define P9_23_GPIO1_17        17
#define GPIO_NUM(cat, no)     (cat*32 + no)

#define PAGE_SIZES            0x1000
#define IO_ADDRESS(addr)      (ioremap(addr, PAGE_SIZES))

#define DATAIN_VALUE(val, no) (1 & (val >> no))


#define INT_BUFF_MAX          64

typedef struct
{
    unsigned long long time;
} __attribute__ ((packed)) R_INT_INFO;

R_INT_INFO intbuffer[INT_BUFF_MAX];
int intcount = 0;

void int_clear(void)
{
    int lp;

    for (lp = 0; lp < INT_BUFF_MAX; lp++)
    {
        intbuffer[lp].time = 0;
    }

    intcount = 0;
}

// parameter 'struct pt_regs *regs' has been removed
irqreturn_t int_interrupt(int irq, void *dev_id)
{
    if (intcount < INT_BUFF_MAX)
    {
        intbuffer[intcount].time = get_jiffies_64();
        intcount++;
    }

    return IRQ_HANDLED;
}

int int_open(struct inode *inode, struct file *filp)
{
    u32 gpio_reg;
    int irqNumber;

    // Set P9_12 as output, P9_23 as input
    gpio_reg = readl(IO_ADDRESS(OE_REG_ADDR));
    gpio_reg &= (0xFFFFFFFF ^ (1 << P9_12_GPIO1_28));
    gpio_reg |= (1 << P9_23_GPIO1_17);

    writel(gpio_reg, IO_ADDRESS(OE_REG_ADDR));

    // initialize interrupt handler
    irqNumber = gpio_to_irq(GPIO_NUM(1, P9_23_GPIO1_17));
    if (!request_irq(irqNumber, int_interrupt, IRQF_TRIGGER_RISING, INT_DEV_NAME, NULL))
    {

    }

    int_clear();

    return 0;
}

ssize_t int_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
    int readcount;
    char *ptrdata;
    int loop;

    readcount = count / sizeof(R_INT_INFO);
    if(readcount > intcount)
        readcount = intcount;

    ptrdata = (char *)&intbuffer[0];

    for (loop = 0; loop < readcount * sizeof(R_INT_INFO); loop++)
    {
        put_user(ptrdata[loop], (char *)&buf[loop]);
    }

    return readcount * sizeof(R_INT_INFO);
}

ssize_t int_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos)
{
    unsigned char status;
    int loop;
    u32 write_val;

    int_clear();

    for (loop = 0; loop < count; loop++)
    {
        write_val = readl(IO_ADDRESS(DATAOUT_REG_ADDR));

        get_user(status, (char *) buf);
        if (status == 0)
        {
            write_val &= (0xFFFFFFFF ^ (1 << P9_12_GPIO1_28));
            writel(write_val, IO_ADDRESS(DATAOUT_REG_ADDR));
        }
        else
        {
           write_val |= (1 << P9_12_GPIO1_28);
           writel(write_val, IO_ADDRESS(DATAOUT_REG_ADDR));
        }
    }

    return count;
}

int int_release(struct inode *inode, struct file *filp)
{
    int irqNumber;

    irqNumber = gpio_to_irq(GPIO_NUM(1, P9_23_GPIO1_17));
    free_irq(irqNumber, NULL);

    return 0;
}

struct file_operations int_fops =
{
    .owner   = THIS_MODULE,
    .read    = int_read,
    .write   = int_write,
    .open    = int_open,
    .release = int_release,
};

int int_init(void)
{
    int result;

    result = register_chrdev(INT_DEV_MAJOR, INT_DEV_NAME, &int_fops);
    if (result < 0)
        return result;

    return 0;
}

void int_exit(void)
{
    unregister_chrdev(INT_DEV_MAJOR, INT_DEV_NAME);
}

module_init(int_init);
module_exit(int_exit);

MODULE_LICENSE("GPL");
