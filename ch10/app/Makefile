CC := gcc
OBJS := ioctl_app.o
TARGET := ioctl_app
CFLAGS := -I../include

.SUFFIXES : .c .o

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) -o $@ $(OBJS) $(CLFAGS)

clean:
	rm -f $(OBJS) $(TARGET)
