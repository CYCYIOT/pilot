#子目录的Makefile直接读取其子目录就行
SUB_DIRS=$(shell ls -l | grep ^d | awk '{print $$9}')
#以下同根目录下的makefile的相同代码的解释

CUR_SOURCE=${wildcard *.c}
CUR_OBJS=${patsubst %.c, %.o, $(CUR_SOURCE)}

all:PRINT $(SUB_DIRS) $(CUR_OBJS)
PRINT:
$(SUB_DIRS):ECHO
	make -C $@ CFLAGS=$(CFLAGS) INC=$(INC)
$(CUR_OBJS):%.o:%.c
	@echo CC $^ -o $@
	@$(CC) $(CFLAGS) $(INC) -c $^ -o $(PROJECT_DIR)$(OBJS_DIR)/$@
ECHO:
	@echo $(SUBDIRS)
