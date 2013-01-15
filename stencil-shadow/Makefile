NAME = stencil-shadow
CC = gcc
CFLAGS = -O3
LDFLAGS = -lm -lrt -lX11 -lGL -lGLU
SRCS = main.c
OBJS=$(SRCS:%.c=%.o)

all: $(NAME)

$(NAME): $(OBJS)
	$(CC) $(LDFLAGS) $(OBJS) -o $(NAME)

.c.o:
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS)
	rm -f $(NAME)
