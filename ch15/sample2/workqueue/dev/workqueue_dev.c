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
#include <linux/wait.h>

#include <linux/workqueue.h>

#define DEV_NAME              "workqueue"
#define DEV_MAJOR             240

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


static int input_enable = 0;
void call_workqueuefunc(struct work_struct *data);

DECLARE_WAIT_QUEUE_HEAD(waitqueue_read);

// changed 'DECLARE_WORK' -> 'DECLARE_DELAYED_WORK'
DECLARE_DELAYED_WORK(work_queue, call_workqueuefunc);

void call_workqueuefunc(struct work_struct *data)
{
    int pinstate;
    u32 status;

    status = readl(IO_ADDRESS(DATAIN_REG_ADDR));
    pinstate = DATAIN_VALUE(status, P9_23_GPIO1_17);

    if (pinstate == 1)
    {
        input_enable = 0;
        wake_up_interruptible(&waitqueue_read);
    }
}

// parameter 'struct pt_regs *regs' has been removed
irqreturn_t workqueue_interrupt(int irq, void *dev_id)
{
    if (input_enable)
    {
        schedule_delayed_work(&work_queue, 100);
    }

    return IRQ_HANDLED;
}

int workqueue_open(struct inode *inode, struct file *filp)
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
    if (!request_irq(irqNumber, workqueue_interrupt, IRQF_TRIGGER_RISING, DEV_NAME, NULL))
    {

    }

    return 0;
}

ssize_t workqueue_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
    input_enable = 1;

    interruptible_sleep_on(&waitqueue_read);

    return 1;
}

ssize_t workqueue_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos)
{
    unsigned char status;
    u32 write_val;

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

    return 1;
}

int workqueue_release(struct inode *inode, struct file *filp)
{
    int irqNumber;

    irqNumber = gpio_to_irq(GPIO_NUM(1, P9_23_GPIO1_17));
    free_irq(irqNumber, NULL);

    return 0;
}

struct file_operations workqueue_fops =
{
    .owner   = THIS_MODULE,
    .read    = workqueue_read,
    .write   = workqueue_write,
    .open    = workqueue_open,
    .release = workqueue_release,
};

int workqueue_init(void)
{
    int result;

    result = register_chrdev(DEV_MAJOR, DEV_NAME, &workqueue_fops);
    if (result < 0)
        return result;

    return 0;
}

void workqueue_exit(void)
{
    unregister_chrdev(DEV_MAJOR, DEV_NAME);
}

module_init(workqueue_init);
module_exit(workqueue_exit);

MODULE_LICENSE("GPL");
