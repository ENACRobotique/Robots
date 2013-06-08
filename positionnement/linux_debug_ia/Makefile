CC   = gcc
SRCS = main.c params.c tools.c perception.c optim.c neldermead.c gradient.c
OBJ  = $(SRCS:.c=.o)
BIN  = pos_test
CFLAGS = -Wall -Werror -g
LIBS = -lm
RM = rm -f
DEPEND = .depend

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

