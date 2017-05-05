CC=g++

SOURCES=$(TARGET).cpp 

OBJECTS=$(SOURCES:.cpp=.o)

EXECUTABLE=$(TARGET)

#CFLAGS := -Wall -g -O3 -Wfatal-errors -mssse3 -msse2 -march=native
CFLAGS := -ggdb 
LDFLAGS :=  CNecklace.cpp #-Llib -Wl,-rpath='$$ORIGIN/lib' -g `pkg-config opencv --libs` 

# OpenCV
#CFLAGS += `pkg-config opencv --cflags`

all: neclage 

neclage: neclage.cpp CNecklace.cpp
	$(CC) neclage.cpp $(CFLAGS) $(LDFLAGS) -o $@ 

clean:
	rm -f $(OBJECTS) $(EXECUTABLE) 

cleanall:
	rm -f *.o 
