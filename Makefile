EXENAME = main

CXX = clang++
CXXFLAGS = $(CS225) -std=c++1y -stdlib=libc++ -c -g -O0 -Wall -Wextra -pedantic
LD = clang++
LDFLAGS = -std=c++1y -stdlib=libc++ -lc++abi -lm

# Custom Clang version enforcement Makefile rule:
ccred=$(shell echo -e "\033[0;31m")
ccyellow=$(shell echo -e "\033[0;33m")
ccend=$(shell echo -e "\033[0m")

IS_EWS=$(shell hostname | grep "ews.illinois.edu")
IS_CORRECT_CLANG=$(shell clang -v 2>&1 | grep "version 6")
ifneq ($(strip $(IS_EWS)),)
ifeq ($(strip $(IS_CORRECT_CLANG)),)
CLANG_VERSION_MSG = $(error $(ccred) On EWS, please run 'module load llvm/6.0.1' first when running CS225 assignments. $(ccend))
endif
else
CLANG_VERSION_MSG = $(warning $(ccyellow) Looks like you are not on EWS. Be sure to test on EWS before the deadline. $(ccend))
endif

.PHONY: all test clean

all : $(EXENAME)

output_msg: ; $(CLANG_VERSION_MSG)

$(EXENAME): output_msg extractData.o dsets.o algorithms.o
	$(LD) main.cpp extractData.o dsets.o algorithms.o graph.o DijkstraVertex.o DijkstraHeap.o $(LDFLAGS) -o $(EXENAME)

extractData.o: graph.o extractData.cpp extractData.h
	$(CXX) $(CXXFLAGS) -c extractData.cpp graph.o

graph.o: include/graph.cpp include/edge.h include/graph.h
	$(CXX) $(CXXFLAGS) -c include/graph.cpp

dsets.o: include/dsets.cpp include/dsets.h
	$(CXX) $(CXXFLAGS) -c include/dsets.cpp

algorithms.o: graph.o DijkstraVertex.o DijkstraHeap.o dsets.o algorithms.cpp algorithms.h
	$(CXX) $(CXXFLAGS) -c algorithms.cpp dsets.o graph.o DijkstraVertex.o DijkstraHeap.o

DijkstraVertex.o: graph.o include/DijkstraVertex.cpp include/DijkstraVertex.h
	$(CXX) $(CXXFLAGS) -c include/DijkstraVertex.cpp graph.o

DijkstraHeap.o: DijkstraVertex.o include/DijkstraHeap.cpp include/DijkstraHeap.h
	$(CXX) $(CXXFLAGS) -c include/DijkstraHeap.cpp DijkstraVertex.o

test: output_msg catch/catchmain.cpp tests/tests.cpp extractData.o algorithms.o
	$(LD) catch/catchmain.cpp tests/tests.cpp extractData.o dsets.o algorithms.o graph.o DijkstraVertex.o DijkstraHeap.o $(LDFLAGS) -o test

clean:
	rm -rf *.o *.dSYM $(EXENAME) test

