CXX ?= g++
AR ?= ar

CXX_STD := -std=c++11

CXXFLAGS ?= -O3 -march=native

bin/mcm: bin/mcm.o lib/libmcm.a
	$(CXX) $(CXX_STD) -DNDEBUG -D_FILE_OFFSET_BITS=64 $(CXXFLAGS) -o $@ $^ -pthread

lib/libmcm.a: lib/Archive.o lib/Huffman.o lib/Memory.o lib/Util.o lib/Compressor.o lib/File.o lib/LZ.o lib/Tests.o
	ar rcs $@ $^

.cpp.o:
	$(CXX) $(CXX_STD) -iquote lib -DNDEBUG -D_FILE_OFFSET_BITS=64 $(CXXFLAGS) -c -o $@ $< -pthread

clean:
	rm -f lib/*.o lib/libmcm.a bin/mcm
