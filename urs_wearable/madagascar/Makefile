# 2012 (C) Jussi Rintanen, jrintanen.jr@gmail.com

# Uncomment for Madagascar with the planning heuristic as default (Mp)
#VERSION = -DMPDOWNLOAD
#EXECUTABLE=Mp
# Uncomment for Madagascar C with the planning heuristic as default (MpC)
VERSION = -DCMPDOWNLOAD
EXECUTABLE=MpC
# Uncomment for Madagascar with VSIDS as default (M)
#VERSION = -DVSIDS
#EXECUTABLE=M
#ARCH=-m32

INSTRUMENT = #-g -ggdb -pg

CONFIGURATION= -DLBD -DREPRTHREE -DWEIGHTS #-DFUIP #-DMULTICORE #-DSPREAD -DCOSTS -DCFMA  -DCP3

#FEATURES= -fopenmp
#LINKFEATURES= -fopenmp

# The following options are specific to GCC. Disable when necessary (e.g. in OS X).
GCCOPTIMIZE= $(ARCH) -O3 -fprefetch-loop-arrays -funroll-loops -ftree-loop-im -ftracer -maccumulate-outgoing-args -momit-leaf-frame-pointer #-falign-functions=64

PARSERGENERATOR=bison
PARSERC = parser.tab.c
PARSERH = parser.tab.h
PARSER=parser
#PARSERGENERATOR=yacc
#PARSERC = y.tab.c
#PARSERH = y.tab.h
#PARSER=y

CC = gcc

#--- For Protocol Buffer ---
PROTO_DIR := ../proto
PROTO_OBJS := action.pb-c.o predicate.pb-c.o state.pb-c.o planning.pb-c.o# order is important!
PROTO_CFLAGS := `pkg-config --cflags 'libprotobuf-c >= 1.0.0'`
PROTO_LFLAGS := `pkg-config --libs 'libprotobuf-c >= 1.0.0'`
#---------------------------

CFLAGS = -c $(VERSION) $(FEATURES) $(CONFIGURATION) $(INSTRUMENT) $(GCCOPTIMIZE) $(PROTO_CFLAGS) -w #-Wall

#LINKFLAGS = -L . -lcp3 -lpthread -lz -lrt #-static
#LINKFLAGS = -s
LINKFLAGS = $(ARCH) $(PROTO_LFLAGS) -lm -static

OBJ = $(PROTO_OBJS) $(PARSER).tab.o main.o asyntax.o lex.yy.o tables.o operators.o invariants.o intsets.o cleanup.o translate2sat.o scc.o clausesets.o printplan.o clausedb.o dimacsinput.o ground.o
HDR = main.h asyntax.h tables.h operators.h invariants.h intsets.h cleanup.h scc.h intsets.h translate2sat.h ordintsets.h clausedb.h interface.h printplan.h dimacsinput.h

all: nplan

nplan: date lex.yy.c $(PARSERC) $(OBJ) $(HDR)
	$(CC) $(OBJ) $(LINKFLAGS) $(LINKFEATURES) $(INSTRUMENT) -o $(EXECUTABLE)

clausesets.o: heuristics2.c varvals.c learn2.c

ground.o: Cground.c

%.o: %.c $(HDR)
	$(CC) $(CFLAGS) $(AMD64FLAGS) $<
	
%.pb-c.o: %.pb-c.c %.pb-c.h
	$(CC) $(CFLAGS) $<

%.pb-c.c %.pb-c.h: $(PROTO_DIR)/%.proto
	protoc-c --proto_path=$(PROTO_DIR) --c_out=. $<

date:
	./makedate

main.o: zPRINTDATE

$(PARSERC) $(PARSERH): parser.y
	$(PARSERGENERATOR) -d parser.y

lex.yy.c: lexer.lex $(PARSERH)
	flex lexer.lex

.PHONY: all nplan date clean tar

clean:
	rm -f $(OBJ) *.pb-c.* lex.yy.c $(PARSERC) $(PARSERH) $(EXECUTABLE) zPRINTDATE

tar:
	tar cvf MADAGASCAR.TAR README plan build asyntax.c asyntax.h clausedb.c clausedb.h clausesets.c clausesets.h cleanup.c cleanup.h dimacs.h dimacsinput.c dimacsinput.h ground.c Cground.c heuristics2.c interface.h intsets.c intsets.h invariants.c invariants.h lexer.lex main.c main.h makedate Makefile operators.c operators.h ordintsets.c ordintsets.h parser.y printplan.c printplan.h scc.c scc.h tables.c tables.h translate2sat.c translate2sat.h varvals.c learn2.c shortcuts.c zPOSTF zPREF
