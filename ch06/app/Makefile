CC := gcc
OBJS := call_app.o
TARGET := call_app

.SUFFIXES : .c .o

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) -o $@ $(OBJS)

clean:
	rm -f $(OBJS) $(TARGET)


