CXX ?= g++
AR ?= ar

CXX_STD := -std=c++11

CXXFLAGS ?= -O3 -flto -march=native -ggdb3 -gdwarf-4

bin/mcm: bin/mcm.o lib/libmcm.a
	$(CXX) $(CXX_STD) -DNDEBUG -D_FILE_OFFSET_BITS=64 $(CXXFLAGS) -o $@ $^ -pthread

bin/mcm.o: bin/mcm.cpp
	$(CXX) $(CXX_STD) -isystem include -iquote lib -DNDEBUG -D_FILE_OFFSET_BITS=64 $(CXXFLAGS) -c -o $@ $< -pthread

lib/libmcm.a: lib/Archive.o lib/Huffman.o lib/Memory.o lib/Util.o lib/Compressor.o lib/File.o lib/Tests.o
	ar rcs $@ $^

%.o: %.cpp %.hpp
	$(CXX) $(CXX_STD) -isystem include -iquote lib -DNDEBUG -D_FILE_OFFSET_BITS=64 $(CXXFLAGS) -c -o $@ $< -pthread

clean:
	rm -f lib/*.o bin/*.o lib/libmcm.a bin/mcm

check: bin/mcm
	./bin/mcm -test -t1 enwiktiny enwiktiny.mcm
