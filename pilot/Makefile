########## cross compiler ##########

PROJECT_DIR  = $(shell pwd)

#export CROSS_COMPILE=$(PROJECT_DIR)../out/sun8iw8p1/linux/common/buildroot/external-toolchain/bin/arm-linux-gnueabi-
export CROSS_COMPILE=$(PROJECT_DIR)../out/sun8iw8p1/linux/common/buildroot/external-toolchain/bin/arm-buildroot-linux-musleabi-

export AS              = $(CROSS_COMPILE)as
export CC              = $(CROSS_COMPILE)gcc
export LD              = $(CROSS_COMPILE)ld
export CPP             = $(CROSS_COMPILE)g++
export AR              = $(CROSS_COMPILE)ar
export NM              = $(CROSS_COMPILE)nm
export STRIP           = $(CROSS_COMPILE)strip
export OBJCOPY         = $(CROSS_COMPILE)objcopy
export OBJDUMP         = $(CROSS_COMPILE)objdump


########### variables  ##########
export PROJECT_DIR := $(PROJECT_DIR)/

export OBJS_DIR=objs
EXCLUDE_DIRS := $(OBJS_DIR) doc out tools

INSTALL_DIR  := ../tools/MR100/ELF
export LIBS = $(shell pwd)/lib


SUB_DIRS := $(shell ls -l | grep ^d | awk '{print $$9}')
SUB_DIRS := $(filter-out $(EXCLUDE_DIRS),$(SUB_DIRS))

INC        := $(addprefix $(PROJECT_DIR),$(SUB_DIRS))
export INC          := $(addprefix -I,$(INC))
export OBJECTS_PATH := $(PROJECT_DIR)/$(OBJS_DIR)
export CFLAGS       :=  -Wall -Werror -Os -L$(LIBS) -lrt -lpthread -fno-strict-aliasing -ldl -lstdc++ -lm -mfloat-abi=softfp -mfpu=neon 


PROTECTED_OBJS := ${patsubst %.c, %.o, $(PROTECTED_FILE)}
 
#最终bin文件的名字，可以更改为自己需要的
BIN=pilot
#bin文件所在的目录
export BIN_DIR = $(shell pwd)

DEBUG =false
#if DEBUG is enabled, then we add debug symbol
ifeq ($(strip $(DEBUG)),true)
	CFLAGS += -g
endif
CFLAGS += -D$(ARCH)
VER ?= 115
CFLAGS += -DVER=$(VER)

CUR_SOURCE=${wildcard *.c}
#将对应的c文件名转为o文件后放在下面的CUR_OBJS变量中
CUR_OBJS=${patsubst %.c, %.o, $(CUR_SOURCE)}
 
#注意这里的顺序，需要先执行SUBDIRS最后才能是DEBUG
all:MKDIR $(SUB_DIRS) $(CUR_OBJS)
	@$(CC) $(CFLAGS) $(INC) -o $(BIN_DIR)/$(BIN) $(OBJECTS_PATH)/*.o 
	@echo -e '\033[0;32;1m'
	@echo "build $(BIN) successfully"
	@echo -e '\033[0m'

MKDIR:
	@mkdir -p $(OBJS_DIR)
	@if [ -d "$(PROJECT_DIR)bin/" ]; then cp -p -u $(PROJECT_DIR)bin/*.o $(OBJS_DIR) ; fi;
	@if [ -d "$(PROJECT_DIR)bin/" ]; then cp -p -u $(PROJECT_DIR)hal/Makefile  $(PROJECT_DIR)bin/ ; fi;

#递归执行子目录下的makefile文件，这是递归执行的关键
$(SUB_DIRS):ECHO
	@$(MAKE) -C $@ 

ECHO:
	@echo
	@echo $(SUB_DIRS)
	@echo
#将c文件编译为o文件，并放在指定放置目标文件的目录中即OBJS_DIR

$(CUR_OBJS):%.o:%.c
	@$(CC) -mcpu=cortex-a7 -mfpu=neon $(CFLAGS) $(INC) -c $^ -o $(PROJECT_DIR)$(OBJS_DIR)/$@

install:
	cp $(BIN_DIR)/$(BIN) $(INSTALL_DIR)/

clean:
	@rm $(OBJS_DIR)/*.o
	@echo $(OBJS_DIR) clean.
	@rm -rf $(BIN_DIR)/$(BIN)
	@echo $(BIN) clean.
