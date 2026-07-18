# gcc auto-detect: Cygwin on Windows, system gcc on Linux CI
CC_CYGWIN = C:/Andestech/BSPv410/cygwin/bin/gcc
CC_SYSTEM = gcc

ifneq ($(OS),Windows_NT)
    CC = $(CC_SYSTEM)
else ifneq ($(wildcard $(CC_CYGWIN)),)
    CC = $(CC_CYGWIN)
else
    CC = $(CC_SYSTEM)
endif

CFLAGS   = -std=gnu99 -Wall -O0 -g -static
INC      = -I. -Iunity

TARGET = test_runner

SRCS  = test_runner.c
SRCS += unity/unity.c

# Self-contained test files (define + test in same file)
SRCS += motor/test_foc_math.c
SRCS += bms/test_bms_smbus.c
SRCS += bms/test_bms_charge.c
SRCS += safety/test_safety_wd.c
SRCS += ota/test_diff.c
SRCS += comm/test_spi.c
SRCS += comm/test_usb.c

.PHONY: all run test clean

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) $(INC) -o $@ $^ -lm

run: $(TARGET)
	@./$(TARGET)

test: $(TARGET)
	@echo ""
	@./$(TARGET)

clean:
	rm -f $(TARGET)
