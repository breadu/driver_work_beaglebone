#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/fcntl.h>

#include <asm/uaccess.h>
#include <asm/io.h>

#define MINOR_DEV_NAME        "minordev"
#define MINOR_DEV_MAJOR       240

// We will use beaglebone P9 hearder 12pin(GPIO1 28) as output,
// and P9 header 23pin as input (GPIO1 17)
//
// Beagle bone GPIO1 register starts from 0x4804C000 (Refer to data sheet).
// OE register sets GPIO pin as input(default 1)/output(0) by setting bit of pin number(0~31).
// DATAOUT register sets value of GPIO pin by setting bit of pin number(0~31).
// DATAIN register gets value of GPIO pin by getting bit of pin number(0~31).
#define GPIO1_ADDR            0x4804C000
#define GPIO1_SIZE            0x1000
#define OE_REG_OFFSET         0x134
#define DATAOUT_REG_OFFSET    0x13C
#define DATAIN_REG_OFFSET     0x138

#define OE_REG_ADDR           (addrIO+OE_REG_OFFSET)
#define DATAOUT_REG_ADDR      (addrIO+DATAOUT_REG_OFFSET)
#define DATAIN_REG_ADDR       (addrIO+DATAIN_REG_OFFSET)

#define P9_12_GPIO1_28        28
#define P9_23_GPIO1_17        17

#define DATAIN_VALUE(val, no) (1 & (val >> no))

static void *addrIO;

int minor0_open(struct inode *inode, struct file *filp)
{
    u32 gpio_reg;

    // Set P9_12 as output
    gpio_reg = readl(OE_REG_ADDR);
    gpio_reg &= (0xFFFFFFFF ^ (1 << P9_12_GPIO1_28));

    writel(gpio_reg, OE_REG_ADDR);

    printk("call minor0_open\n");

    return 0;
}

ssize_t minor0_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos)
{
    unsigned char status;
    u32 write_val;

    write_val = readl(DATAOUT_REG_ADDR);

    get_user(status, (char *) buf);
    if (status == 0)
    {
        write_val &= (0xFFFFFFFF ^ (1 << P9_12_GPIO1_28));
        writel(write_val, DATAOUT_REG_ADDR);
    }
    else
    {
        write_val |= (1 << P9_12_GPIO1_28);
        writel(write_val, DATAOUT_REG_ADDR);
    }

    return 1;
}

int minor0_release(struct inode *inode, struct file *filp)
{
    printk("call minor0_release\n");
    return 0;
}

int minor1_open(struct inode *inode, struct file *filp)
{
    u32 gpio_reg;

    // Set P9_23 as input
    gpio_reg = readl(OE_REG_ADDR);
    gpio_reg |= (1 << P9_23_GPIO1_17);

    writel(gpio_reg, OE_REG_ADDR);

    printk("call minor1_open\n");

    return 0;
}

ssize_t minor1_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
    u32 status;

    status = readl(DATAIN_REG_ADDR);
    put_user(DATAIN_VALUE(status, P9_23_GPIO1_17), &buf[0]);

    return 1;
}

int minor1_release(struct inode *inode, struct file *filp)
{
    printk("call minor1_release\n");
    return 0;
}

struct file_operations minor0_fops =
{
    .owner   = THIS_MODULE,
    .write   = minor0_write,
    .open    = minor0_open,
    .release = minor0_release,
};

struct file_operations minor1_fops =
{
    .owner   = THIS_MODULE,
    .read    = minor1_read,
    .open    = minor1_open,
    .release = minor1_release,
};

int minor_open(struct inode *inode, struct file *filp)
{
    printk("call minor_open\n");
    switch(MINOR(inode->i_rdev))
    {
    case 1:
        filp->f_op = &minor0_fops;
        break;

    case 2:
        filp->f_op = &minor1_fops;
        break;

    default:
        return -ENXIO;
    }

    if (filp->f_op && filp->f_op->open)
        return filp->f_op->open(inode, filp);

    return 0;
}

struct file_operations minor_fops =
{
    .owner   = THIS_MODULE,
    .open    = minor_open,
};

int minor_init(void)
{
    int result;

    result = register_chrdev(MINOR_DEV_MAJOR, MINOR_DEV_NAME, &minor_fops);
    if (result < 0)
        return result;

    // mapping physical memory to virtual memory
    addrIO = ioremap(GPIO1_ADDR, GPIO1_SIZE);

    return 0;
}

void minor_exit(void)
{
    unregister_chrdev(MINOR_DEV_MAJOR, MINOR_DEV_NAME);

    // unmapping memory
    iounmap(addrIO);
}

module_init(minor_init);
module_exit(minor_exit);

MODULE_LICENSE("GPL");
