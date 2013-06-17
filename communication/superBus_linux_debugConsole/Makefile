CSRCS   = superBus_console.c
CFLAGS  = -Wall -Werror -g
LDFLAGS = 
BIN     = console

# external ressources
COMM    = ..
CFLAGS  += -I. -I$(COMM)/superBus_shared -I$(COMM)/superBus_linux
CSRCS   += $(COMM)/superBus_shared/lib_checksum.c \
           $(COMM)/superBus_shared/lib_superBus.c \
           $(COMM)/superBus_shared/messages.c \
           $(COMM)/superBus_shared/network_cfg.c \
           $(COMM)/superBus_linux/lib_Xbee_x86.c

# /!\ If you're not sure, do not edit past this line!
CC      = gcc
COBJS   = $(CSRCS:.c=.o)
RM      = rm -f
DEPEND  = .depend

.PHONY: all clean depend

all: $(BIN)

clean:
	@echo RM
	@$(RM) $(COBJS) $(BIN)

$(BIN): $(COBJS)
	@echo LD $@
	@$(CC) $(COBJS) -o $(BIN) $(LDFLAGS)

%.o: %.c
	@echo CC $<
	@$(CC) -c $< -o $@ $(CFLAGS)

# auto dependencies
depend: $(DEPEND)

$(DEPEND): $(CSRCS)
	@echo CC $(DEPEND)
	@$(CC) $(CFLAGS) -MM $^ > ./$(DEPEND)

-include $(DEPEND)

