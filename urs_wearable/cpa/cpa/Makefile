# Makefile

OBJS = bison.o lex.o main.o reader.o action.o planner.o state.o \
       protobuf_helper.o planning.pb.o state.pb.o predicate.pb.o action.pb.o

CC := g++
CFLAGS := -c -std=c++11 `pkg-config --cflags protobuf` -Wall -O3
LFLAGS := -lfl `pkg-config --libs protobuf`
PBDIR := ../../include/proto_generated

cpa+: $(OBJS)
	$(CC) $(OBJS) $(LFLAGS) -o cpa+

lex.o: lex.c
	$(CC) $(CFLAGS) -o lex.o lex.c

lex.c: lcp.lex 
	flex lcp.lex
	mv lex.yy.c lex.c

bison.o: bison.c
	$(CC) $(CFLAGS) -o bison.o bison.c

bison.c: lcp.y
	bison -d -v lcp.y
	cp lcp.tab.c bison.c
	cmp -s lcp.tab.h tok.h || cp lcp.tab.h tok.h

main.o:	main.cpp planner.h define.h
	echo "#define BUILT_DATE \"`date`\"" > built_date
	cat built_date main.cpp > main.temp.cpp
	$(CC) $(CFLAGS) -o main.o main.temp.cpp
	rm main.temp.cpp

reader.o: reader.cpp reader.h define.h
	$(CC) $(CFLAGS) -o reader.o reader.cpp

action.o: action.cpp action.h define.h
	$(CC) $(CFLAGS) -o action.o action.cpp

state.o: state.cpp state.h define.h planner.h
	$(CC) $(CFLAGS) -o state.o state.cpp

planner.o: planner.cpp planner.h state.h define.h reader.h
	$(CC) $(CFLAGS) -o planner.o planner.cpp

protobuf_helper.o: ../../src/protobuf_helper.cpp ../../include/protobuf_helper.h
	$(CC) $(CFLAGS) -I../../include -o protobuf_helper.o ../../src/protobuf_helper.cpp

planning.pb.o: $(PBDIR)/planning.pb.cc $(PBDIR)/planning.pb.h
	$(CC) $(CFLAGS) -I$(PBDIR) -o planning.pb.o $(PBDIR)/planning.pb.cc

state.pb.o: $(PBDIR)/state.pb.cc $(PBDIR)/state.pb.h
	$(CC) $(CFLAGS) -I$(PBDIR) -o state.pb.o $(PBDIR)/state.pb.cc

predicate.pb.o: $(PBDIR)/predicate.pb.cc $(PBDIR)/predicate.pb.h
	$(CC) $(CFLAGS) -I$(PBDIR) -o predicate.pb.o $(PBDIR)/predicate.pb.cc

action.pb.o: $(PBDIR)/action.pb.cc $(PBDIR)/action.pb.h
	$(CC) $(CFLAGS) -I$(PBDIR) -o action.pb.o $(PBDIR)/action.pb.cc

clean:
	rm -f *.o *~ lex.c bison.c tok.h lcp.tab.c lcp.tab.h lcp.output cpa+
