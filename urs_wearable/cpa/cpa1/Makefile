# Makefile

CC := g++

PROTO_DIR := ../../proto
PROTO_HELPER_SRC_DIR := ../../src
PROTO_HELPER_HDR_DIR := ../../include
PROTO_OBJS := action.pb.o predicate.pb.o state.pb.o planning.pb.o# order is important!
PROTO_CFLAGS := `pkg-config --cflags protobuf`
PROTO_LFLAGS := `pkg-config --libs protobuf`

CFLAGS := -c -std=c++11 $(PROTO_CFLAGS) -Wall -O3
LFLAGS := -lfl $(PROTO_LFLAGS)

OBJS := protobuf_helper.o $(PROTO_OBJS) bison.o lex.o main.o reader.o action.o planner.o state.o
HDR := action.h define.h planner.h reader.h state.h

cpa+: $(OBJS)
	$(CC) $(OBJS) $(LFLAGS) -o cpa+

lex.o: lex.c
	$(CC) $(CFLAGS) $<

lex.c: lcp.lex 
	flex lcp.lex
	mv lex.yy.c lex.c

bison.o: bison.c
	$(CC) $(CFLAGS) $<

bison.c: lcp.y
	bison -d -v lcp.y
	cp lcp.tab.c bison.c
	cmp -s lcp.tab.h tok.h || cp lcp.tab.h tok.h

main.o:	main.cpp planner.h define.h
	echo "#define BUILT_DATE \"`date`\"" > built_date
	cat built_date main.cpp > main.temp.cpp
	$(CC) $(CFLAGS) -o main.o main.temp.cpp
	rm main.temp.cpp

%.o: %.cpp $(HDR)
	$(CC) $(CFLAGS) $<

%.pb.o: %.pb.cc %.pb.h
	$(CC) $(CFLAGS) $<

%.pb.cc %.pb.h: $(PROTO_DIR)/%.proto
	protoc --proto_path=$(PROTO_DIR) --cpp_out=. $<

protobuf_helper.o: $(PROTO_HELPER_SRC_DIR)/protobuf_helper.cpp $(PROTO_HELPER_HDR_DIR)/protobuf_helper.h
	$(CC) $(CFLAGS) -I$(PROTO_HELPER_HDR_DIR) $<

clean:
	rm -f *.o *~ *.pb.* lex.c bison.c tok.h lcp.tab.c lcp.tab.h lcp.output cpa+
