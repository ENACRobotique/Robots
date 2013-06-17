CSRCS   = main.c params.c tools.c perception.c optim.c neldermead.c gradient.c
CFLAGS  = -Wall -g
LDFLAGS = 
BIN     = pos_test

# external ressources
LDFLAGS += -lm

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

