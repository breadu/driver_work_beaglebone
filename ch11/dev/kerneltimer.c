#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/fcntl.h>

#include <asm/uaccess.h>
#include <asm/io.h>

#include <linux/slab.h>

#include <linux/time.h>
#include <linux/timer.h>

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

#define PAGE_SIZES            0x1000
#define IO_ADDRESS(addr)      (ioremap(addr, PAGE_SIZES))

#define DATAIN_VALUE(val, no) (1 & (val >> no))


#define TIME_STEP     (2*HZ/10)

typedef struct
{
    struct timer_list timer;
    unsigned char led;
} __attribute__ ((packed)) KERNEL_TIMER_MANAGER;

static KERNEL_TIMER_MANAGER *ptrmng = NULL;

void kerneltimer_timeover(unsigned long arg);

void kerneltimer_registertimer(KERNEL_TIMER_MANAGER *pdata, unsigned long timeover)
{
    init_timer(&(pdata->timer));

    pdata->timer.expires  = get_jiffies_64() + timeover;
    pdata->timer.data     = (unsigned long)pdata;
    pdata->timer.function = kerneltimer_timeover;

    add_timer(&(pdata->timer));
}

void kerneltimer_timeover(unsigned long arg)
{
    KERNEL_TIMER_MANAGER *pdata = NULL;
    u32 write_val;

    if (arg)
    {
        pdata = (KERNEL_TIMER_MANAGER *)arg;

        write_val = readl(IO_ADDRESS(DATAOUT_REG_ADDR));

        if (pdata->led == 0)
        {
            write_val &= (0xFFFFFFFF ^ (1 << P9_12_GPIO1_28));
            writel(write_val, IO_ADDRESS(DATAOUT_REG_ADDR));
        }
        else
        {
            write_val |= (1 << P9_12_GPIO1_28);
            writel(write_val, IO_ADDRESS(DATAOUT_REG_ADDR));
        }

        pdata->led = ~pdata->led & 0x01;

        kerneltimer_registertimer(pdata, TIME_STEP);
    }
}

int kerneltimer_init(void)
{
    u32 gpio_reg;

    // Set P9_12 as output
    gpio_reg = readl(IO_ADDRESS(OE_REG_ADDR));
    gpio_reg &= (0xFFFFFFFF ^ (1 << P9_12_GPIO1_28));

    writel(gpio_reg, IO_ADDRESS(OE_REG_ADDR));

    // initialize kernel timer manager
    ptrmng = kmalloc(sizeof(KERNEL_TIMER_MANAGER), GFP_KERNEL);
    if (ptrmng == NULL)
        return -ENOMEM;

    memset(ptrmng, 0, sizeof(KERNEL_TIMER_MANAGER));

    ptrmng->led = 0;
    kerneltimer_registertimer(ptrmng, TIME_STEP);

    return 0;
}

void kerneltimer_exit(void)
{
    u32 write_val;

    if (ptrmng != NULL)
    {
        del_timer(&(ptrmng->timer));
        kfree(ptrmng);
    }

    write_val = readl(IO_ADDRESS(DATAOUT_REG_ADDR));
    write_val &= (0xFFFFFFFF ^ (1 << P9_12_GPIO1_28));

    writel(write_val, IO_ADDRESS(DATAOUT_REG_ADDR));
}

module_init(kerneltimer_init);
module_exit(kerneltimer_exit);

MODULE_LICENSE("GPL");
