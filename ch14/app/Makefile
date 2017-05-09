CC := gcc
OBJS := poll_app.o
TARGET := poll_app

.SUFFIXES : .c .o

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) -o $@ $(OBJS)

clean:
	rm -f $(OBJS) $(TARGET)


