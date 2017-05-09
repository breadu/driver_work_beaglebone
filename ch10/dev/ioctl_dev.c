#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/fcntl.h>

#include <asm/uaccess.h>
#include <asm/io.h>

#include "ioctl_test.h"

#define IOCTLTEST_DEV_NAME    "ioctldev"
#define IOCTLTEST_DEV_MAJOR   240

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

int ioctltest_open(struct inode *inode, struct file *filp)
{
    u32 gpio_reg;

    // Set P9_12 as output, P9_23 as input
    gpio_reg = readl(IO_ADDRESS(OE_REG_ADDR));
    gpio_reg &= (0xFFFFFFFF ^ (1 << P9_12_GPIO1_28));
    gpio_reg |= (1 << P9_23_GPIO1_17);

    writel(gpio_reg, IO_ADDRESS(OE_REG_ADDR));

    return 0;
}

int ioctltest_release(struct inode *inode, struct file *filp)
{
    return 0;
}

// 'ioctl' field in file_operations changed into 'unlocked_ioctl' from kernel 2.6.38
// first parameter 'struct inode* inode' has been removed from ioctl callback function
long ioctltest_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    ioctl_test_info ctrl_info;
    int err, size;
    int loop;
    u32 write_val;
    u32 status;

    if (_IOC_TYPE(cmd) != IOCTLTEST_MAGIC)
        return -EINVAL;

    if (_IOC_NR(cmd) >= IOCTLTEST_MAXNR)
        return -EINVAL;

    size = _IOC_SIZE(cmd);

    if (size > 0)
    {
        err = 0;
        // changed 'verify_area' -> 'access_ok'
        if (_IOC_DIR(cmd) & _IOC_READ)
            err = access_ok(VERIFY_WRITE, (void *)arg, size);
        else if (_IOC_DIR(cmd) & _IOC_WRITE)
            err = access_ok(VERIFY_READ, (void *)arg, size);

        // 'access_ok()' returns true(nonzero) if the memory block may be valid,
        // false(zero) if it is definitely invalid
        if (!err)
            return -EINVAL;
    }

    switch(cmd)
    {
    case IOCTLTEST_LEDOFF:
        write_val = readl(IO_ADDRESS(DATAOUT_REG_ADDR));
        write_val &= (0xFFFFFFFF ^ (1 << P9_12_GPIO1_28));

        writel(write_val, IO_ADDRESS(DATAOUT_REG_ADDR));
        break;

    case IOCTLTEST_LEDON:
        write_val = readl(IO_ADDRESS(DATAOUT_REG_ADDR));
        write_val |= (1 << P9_12_GPIO1_28);

        writel(write_val, IO_ADDRESS(DATAOUT_REG_ADDR));
        break;

    case IOCTLTEST_GETSTATE:
        status = readl(IO_ADDRESS(DATAIN_REG_ADDR));
        return DATAIN_VALUE(status, P9_23_GPIO1_17);

    case IOCTLTEST_READ:
        status = readl(IO_ADDRESS(DATAIN_REG_ADDR));

        ctrl_info.buff[0] = DATAIN_VALUE(status, P9_23_GPIO1_17);
        ctrl_info.size = 1;

        copy_to_user((void *)arg, (const void *)&ctrl_info, (unsigned long)size);
        break;

    case IOCTLTEST_WRITE:
        copy_from_user((void *)&ctrl_info, (const void *)arg, size);

        for (loop = 0; loop < ctrl_info.size; loop++)
        {
            write_val = readl(IO_ADDRESS(DATAOUT_REG_ADDR));

            if (ctrl_info.buff[loop] == 0)
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
        break;

    case IOCTLTEST_WRITE_READ:
        copy_from_user((void *)&ctrl_info, (const void *)arg, size);

        for (loop = 0; loop < ctrl_info.size; loop++)
        {
            write_val = readl(IO_ADDRESS(DATAOUT_REG_ADDR));

            if (ctrl_info.buff[loop] == 0)
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

        status = readl(IO_ADDRESS(DATAIN_REG_ADDR));

        ctrl_info.buff[0] = DATAIN_VALUE(status, P9_23_GPIO1_17);
        ctrl_info.size = 1;

        copy_to_user((void *)arg, (const void *)&ctrl_info, (unsigned long)size);
        break;
    }

    return 0;
}

// 'ioctl' field in file_operations changed into 'unlocked_ioctl' from kernel 2.6.38
// first parameter 'struct inode* inode' has been removed from ioctl callback function
struct file_operations ioctltest_fops =
{
    .owner   = THIS_MODULE,
    .unlocked_ioctl   = ioctltest_ioctl,
    .open    = ioctltest_open,
    .release = ioctltest_release,
};

ssize_t rdwr_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
    u32 status;

    status = readl(IO_ADDRESS(DATAIN_REG_ADDR));
    put_user(DATAIN_VALUE(status, P9_23_GPIO1_17), &buf[0]);

    return 1;
}

ssize_t rdwr_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos)
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

int ioctltest_init(void)
{
    int result;

    result = register_chrdev(IOCTLTEST_DEV_MAJOR, IOCTLTEST_DEV_NAME, &ioctltest_fops);
    if (result < 0)
        return result;

    return 0;
}

void ioctltest_exit(void)
{
    unregister_chrdev(IOCTLTEST_DEV_MAJOR, IOCTLTEST_DEV_NAME);
}

module_init(ioctltest_init);
module_exit(ioctltest_exit);

MODULE_LICENSE("GPL");
