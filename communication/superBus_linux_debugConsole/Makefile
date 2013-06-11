SRCS    = superBus_console.c
CFLAGS  = -Wall -Werror -g 
LDFLAGS = 
BIN     = console

# external ressources
CFLAGS  += -I. -I../superBus_shared -I../superBus_linux
SRCS    += ../superBus_shared/lib_checksum.c \
           ../superBus_shared/lib_superBus.c \
           ../superBus_shared/messages.c \
           ../superBus_shared/network_cfg.c \
           ../superBus_linux/lib_Xbee_x86.c

# /!\ If you're not sure, do not edit past this line /!\
CC      = gcc
OBJ     = $(SRCS:.c=.o)
RM      = rm -f
DEPEND  = .depend

.PHONY: all clean depend

all: $(BIN)

clean:
	${RM} $(OBJ) $(BIN)

$(BIN): $(OBJ)
	$(CC) $(OBJ) -o $(BIN) $(LIBS)

%.o: %.c
	$(CC) -c $< -o $@ $(CFLAGS)

# auto dependencies
depend: $(DEPEND)

$(DEPEND): $(SRCS)
	$(CC) $(CFLAGS) -MM $^ > ./$(DEPEND)

-include $(DEPEND)

