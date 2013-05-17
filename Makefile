CC   = gcc
SRCS = main_test_astar.c tools.c math_ops.c a_star.c
OBJ  = $(SRCS:.c=.o)
BIN  = main_test_astar
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

