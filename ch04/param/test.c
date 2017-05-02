#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/moduleparam.h>

static int oneValue = 1;
static char *twoString = NULL;

module_param(oneValue, int, 0);
module_param(twoString, charp, 0);

static int hello_init(void)
{
    printk("Hello, world [onevalue=%d:twostring=%s]\n",
            oneValue, twoString);
    return 0;
}

static void hello_exit(void)
{
    printk("Goodbye, world\n");
}

module_init(hello_init);
module_exit(hello_exit);

MODULE_AUTHOR("Changbae Bang");
MODULE_DESCRIPTION("Module Parameter Test Module");
MODULE_LICENSE("GPL");
