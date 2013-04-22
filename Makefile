# Project: tangents_test
# Makefile created by Dev-C++ 4.9.9.2

CC   = gcc
OBJ  = main.o tools.o math_ops.o
BIN  = tangents_test
CFLAGS = -O3 -Wall #-Werror
LIBS = -lm
RM = rm -f

.PHONY: all clean

all: tangents_test


clean:
	${RM} $(OBJ) $(BIN)

$(BIN): $(OBJ)
	$(CC) $(OBJ) -o $(BIN) $(LIBS)

%.o: %.c
	$(CC) -c $^ -o $@ $(CFLAGS)

