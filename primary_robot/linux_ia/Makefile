CSRCS   = a_star.c math_ops.c tools.c main_debug_ia.c
CFLAGS  = -Wall -g
LDFLAGS = 
BIN     = debug_ia

# external ressources
COMM    = ../../communication
CFLAGS  += -I. -I$(COMM)/superBus_shared -I$(COMM)/superBus_linux
CSRCS   += $(COMM)/superBus_shared/lib_checksum.c \
           $(COMM)/superBus_shared/lib_superBus.c \
           $(COMM)/superBus_shared/messages.c \
           $(COMM)/superBus_shared/network_cfg.c \
           $(COMM)/superBus_linux/lib_Xbee_x86.c
LDFLAGS += -lm

# /!\ If you're not sure, do not edit past this line!
CC      = gcc
COBJS   = $(CSRCS:.c=.o)
RM      = rm -f
DEPEND  = .depend

.PHONY: all clean depend

all: $(BIN)

clean:
	@echo RM '*.o' $(BIN)
	@$(RM) $(COBJS) $(BIN)

$(BIN): $(COBJS)
	@echo LD -o $@
	@$(CC) $^ -o $@ $(LDFLAGS)

%.o: %.c
	@echo CC -o $@
	@$(CC) -c $< -o $@ $(CFLAGS)

# auto dependencies
depend: $(DEPEND)

$(DEPEND): $(CSRCS)
	@echo DEP -o $@
	@$(CC) $(CFLAGS) -MM $^ > ./$@

-include $(DEPEND)

