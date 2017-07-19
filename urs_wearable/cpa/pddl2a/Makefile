# Makefile

OBJS = bison.o lex.o main.o define.o reader.o action.o planner.o state.o

CC = g++
CFLAGS = -Wall -std=c++11 -O3

cpa:		$(OBJS)
		$(CC) $(CFLAGS) $(OBJS) -o cpa.pddl2pl -lfl

lex.o:		lex.c
		$(CC) $(CFLAGS) -c lex.c -o lex.o

lex.c:		lcp.lex 
		flex lcp.lex
		cp lex.yy.c lex.c

bison.o:	bison.c
		$(CC) $(CFLAGS) -c bison.c -o bison.o

bison.c:	lcp.y
		bison -d -v lcp.y
		cp lcp.tab.c bison.c
		cmp -s lcp.tab.h tok.h || cp lcp.tab.h tok.h

main.o:	main.cpp planner.h define.h
		$(CC) $(CFLAGS) -c main.cpp -o main.o

define.o: define.cpp define.h
	$(CC) $(CFLAGS) -c define.cpp -o define.o

reader.o: reader.cpp reader.h  define.h
	$(CC) $(CFLAGS) -c reader.cpp -o reader.o

action.o: action.cpp action.h  define.h
	$(CC) $(CFLAGS) -c action.cpp -o action.o

state.o: state.cpp state.h  define.h
	$(CC) $(CFLAGS) -c state.cpp -o state.o

planner.o: planner.cpp planner.h  state.h define.h
	$(CC) $(CFLAGS) -c planner.cpp -o planner.o

lex.o yac.o main.o	: 
lex.o main.o		: 

clean:
	rm -f *.o *~ lex.c lex.yy.c bison.c tok.h lcp.tab.c lcp.tab.h lcp.output cpa

