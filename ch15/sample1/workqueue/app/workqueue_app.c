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
    char buff[128];

    printf("workqueue test start\n");

    dev = open(DEVICE_FILENAME, O_RDWR);
    if (dev < 0)
    {
        printf("failed to open device!\n");
        exit(-1);
    }

    printf("read wait\n");
    read(dev, buff, 1);

    printf("write 1\n");
    write(dev, buff, 1);

    printf("write 2\n");
    write(dev, buff, 1);

    printf("Program end\n");
    close(dev);

    return 0;
}
