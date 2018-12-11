TARGET = retrogram-plutosdr
LIBS = -lncurses -lboost_program_options -liio
CXX = g++
CXXFLAGS = -g -Wall -std=c++11

TARGET_STATIC_ARM = retrogram-plutosdr.arm
CXX_ARM = g++
CXXFLAGS_STATIC_ARM = -Wl,-static -lncurses -Wl,-static -lboost_program_options -Wl,-static -ltinfo -Wl,-static -lgpm -Wl,-Bdynamic -liio -std=c++11

.PHONY: default all clean

default: $(TARGET)
all: default

OBJECTS = retrogram-plutosdr.cpp
HEADERS = ascii_art_dft.hpp

%.o: %.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) -c $< -o $@

.PRECIOUS: $(TARGET) $(OBJECTS)

$(TARGET): $(OBJECTS)
	$(CXX) $(OBJECTS) $(CXXFLAGS) $(LIBS) -o $@

arm:
	$(CXX_ARM) $(OBJECTS) $(CXXFLAGS_STATIC_ARM) -o $(TARGET_STATIC_ARM)

clean:
	-rm -f *.o
	-rm -f $(TARGET)
