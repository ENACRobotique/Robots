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

