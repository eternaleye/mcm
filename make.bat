g++ -std=c++11 -iquote lib -DNDEBUG -DWIN32 -D_FILE_OFFSET_BITS=64 -O3 -march=native -o bin\mcm bin\mcm.cpp lib\Archive.cpp lib\Huffman.cpp lib\Memory.cpp lib\Util.cpp lib\Compressor.cpp lib\File.cpp lib\LZ.cpp lib\Tests.cpp -pthread
PAUSE
