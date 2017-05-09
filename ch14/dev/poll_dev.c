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

#include <linux/poll.h>

#define POLL_DEV_NAME    "polldev"
#define POLL_DEV_MAJOR   240

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


#define POLL_BUFF_MAX         64

DECLARE_WAIT_QUEUE_HEAD(WaitQueue_Read);

#define MAX_QUEUE_CNT         128

static unsigned char ReadQ[MAX_QUEUE_CNT];
static unsigned long ReadQCount = 0;
static unsigned long ReadQHead = 0;
static unsigned long ReadQTail = 0;

// parameter 'struct pt_regs *regs' has been removed
irqreturn_t poll_interrupt(int irq, void *dev_id)
{
    unsigned long flags;
    u32 status;

    local_save_flags(flags);
    local_irq_disable();

    if (ReadQCount < MAX_QUEUE_CNT)
    {
        status = readl(IO_ADDRESS(DATAIN_REG_ADDR));

        ReadQ[ReadQHead] = DATAIN_VALUE(status, P9_23_GPIO1_17);
        ReadQHead = (ReadQHead + 1) % MAX_QUEUE_CNT;
        ReadQCount++;
    }

    local_irq_restore(flags);

    wake_up_interruptible(&WaitQueue_Read);

    return IRQ_HANDLED;
}

int poll_open(struct inode *inode, struct file *filp)
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
    if (!request_irq(irqNumber, poll_interrupt, IRQF_TRIGGER_RISING, POLL_DEV_NAME, NULL))
    {

    }

    return 0;
}

ssize_t poll_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
    unsigned long flags;
    int realmax;
    int loop;
    int retstate;

    if ((ReadQCount == 0) && (filp->f_flags & O_NONBLOCK))
        return -EAGAIN;

    retstate = wait_event_interruptible(WaitQueue_Read, ReadQCount);

    if (retstate)
        return retstate;

    local_save_flags(flags);
    local_irq_disable();

    realmax = 0;
    if (ReadQCount > 0)
    {
        if (ReadQCount <= count)
            realmax = ReadQCount;
        else
            realmax = count;

        for (loop = 0; loop < realmax; loop++)
        {
            put_user(ReadQ[ReadQTail], (char *)&buf[loop]);
            ReadQTail = (ReadQTail + 1) % MAX_QUEUE_CNT;
            ReadQCount--;
        }
    }

    local_irq_restore(flags);

    return realmax;
}

ssize_t poll_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos)
{
    unsigned char status;
    int loop;
    u32 write_val;

    for (loop = 0; loop < count; loop++)
    {
        write_val = readl(IO_ADDRESS(DATAOUT_REG_ADDR));

        get_user(status, (char *) &buf[loop]);
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

unsigned int poll_poll(struct file *filp, poll_table *wait)
{
    unsigned int mask = 0;

    poll_wait(filp, &WaitQueue_Read, wait);

    if (ReadQCount > 0)
        mask |= POLLIN | POLLRDNORM;

    return mask;
}

int poll_release(struct inode *inode, struct file *filp)
{
    int irqNumber;

    irqNumber = gpio_to_irq(GPIO_NUM(1, P9_23_GPIO1_17));
    free_irq(irqNumber, NULL);

    return 0;
}

struct file_operations poll_fops =
{
    .owner   = THIS_MODULE,
    .read    = poll_read,
    .write   = poll_write,
    .poll    = poll_poll,
    .open    = poll_open,
    .release = poll_release,
};

int poll_init(void)
{
    int result;

    result = register_chrdev(POLL_DEV_MAJOR, POLL_DEV_NAME, &poll_fops);
    if (result < 0)
        return result;

    return 0;
}

void poll_exit(void)
{
    unregister_chrdev(POLL_DEV_MAJOR, POLL_DEV_NAME);
}

module_init(poll_init);
module_exit(poll_exit);

MODULE_LICENSE("GPL");
