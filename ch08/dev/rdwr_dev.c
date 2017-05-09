#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/fcntl.h>

#include <asm/uaccess.h>
#include <asm/io.h>

#define RDWR_DEV_NAME         "rdwrdev"
#define RDWR_DEV_MAJOR        240

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

int rdwr_open(struct inode *inode, struct file *filp)
{
    u32 gpio_reg;

    // Set P9_12 as output, P9_23 as input
    gpio_reg = readl(OE_REG_ADDR);
    gpio_reg &= (0xFFFFFFFF ^ (1 << P9_12_GPIO1_28));
    gpio_reg |= (1 << P9_23_GPIO1_17);

    writel(gpio_reg, OE_REG_ADDR);

    return 0;
}

ssize_t rdwr_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
    u32 status;

    status = readl(DATAIN_REG_ADDR);
    put_user(DATAIN_VALUE(status, P9_23_GPIO1_17), &buf[0]);

    return 1;
}

ssize_t rdwr_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos)
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

int rdwr_release(struct inode *inode, struct file *filp)
{
    return 0;
}

struct file_operations rdwr_fops =
{
    .owner   = THIS_MODULE,
    .read    = rdwr_read,
    .write   = rdwr_write,
    .open    = rdwr_open,
    .release = rdwr_release,
};

int rdwr_init(void)
{
    int result;

    result = register_chrdev(RDWR_DEV_MAJOR, RDWR_DEV_NAME, &rdwr_fops);
    if (result < 0)
        return result;

    // mapping physical memory to virtual memory
    addrIO = ioremap(GPIO1_ADDR, GPIO1_SIZE);

    return 0;
}

void rdwr_exit(void)
{
    unregister_chrdev(RDWR_DEV_MAJOR, RDWR_DEV_NAME);

    // unmapping memory
    iounmap(addrIO);
}

module_init(rdwr_init);
module_exit(rdwr_exit);

MODULE_LICENSE("GPL");
