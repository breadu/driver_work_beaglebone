#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

#define DEVICE_FILENAME  "/dev/workqueue"

int main()
{
    int dev;
    int lp;
    char buff[128];

    dev = open(DEVICE_FILENAME, O_RDWR);
    if (dev < 0)
    {
        printf("failed to open device!\n");
        exit(-1);
    }

    buff[0] = 0x00;
    write(dev, buff, 1);

    printf("read wait\n");
    for (lp = 0; lp < 3; lp++)
    {
        read(dev, buff, 1);
        printf("check input\n");
    }

    printf("led flashing\n");
    for (lp = 0; lp < 3; lp++)
    {
        buff[0] = 0x01;
        write(dev, buff, 1);
        sleep(1);

        buff[0] = 0x00;
        write(dev, buff, 1);
        sleep(1);
    }

    close(dev);

    return 0;
}
