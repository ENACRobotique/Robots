CC   = gcc
SRCS = main.c tools.c math_ops.c
OBJ  = $(SRCS:.c=.o)
BIN  = tangents_test
CFLAGS = -Wall -Werror -g
LIBS = -lm
RM = rm -f
DEPEND = .depend

.PHONY: all clean depend

all: tangents_test

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

