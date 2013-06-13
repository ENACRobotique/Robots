SRCS    = a_star.c math_ops.c tools.c main_debug_ia.c
CFLAGS  = -Wall -g
LDFLAGS = -lm
BIN     = debug_ia

# external ressources
COMM    = ../../communication
CFLAGS  += -I. -I${COMM}/superBus_shared -I${COMM}/superBus_linux
SRCS    += ${COMM}/superBus_shared/lib_checksum.c \
           ${COMM}/superBus_shared/lib_superBus.c \
           ${COMM}/superBus_shared/messages.c \
           ${COMM}/superBus_shared/network_cfg.c \
           ${COMM}/superBus_linux/lib_Xbee_x86.c

# /!\ If you're not sure, do not edit past this line /!\
CC      = gcc
OBJ     = $(SRCS:.c=.o)
RM      = rm -f
DEPEND  = .depend

.PHONY: all clean depend

all: $(BIN)

clean:
	@${RM} $(OBJ) $(BIN)
	@echo RM

$(BIN): $(OBJ)
	@$(CC) $(OBJ) -o $(BIN) $(LDFLAGS)
	@echo LD $@

%.o: %.c
	@$(CC) -c $< -o $@ $(CFLAGS)
	@echo CC $<

# auto dependencies
depend: $(DEPEND)

$(DEPEND): $(SRCS)
	@$(CC) $(CFLAGS) -MM $^ > ./$(DEPEND)
	@echo CC $(DEPEND)

-include $(DEPEND)

