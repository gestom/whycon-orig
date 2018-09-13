CC=g++

SOURCES=$(TARGET).cpp 

OBJECTS=$(SOURCES:.cpp=.o)

EXECUTABLE=$(TARGET)

CFLAGS := -ggdb -g -std=c++11 -Wall -O2 -pthread
CFLAGS += -fno-strict-aliasing -I/usr/include/Magick
LDFLAGS := CNecklace.cpp -lm -lpthread
LDFLAGS += -lGraphicsMagick++ # -lpng12 -L/usr/lib -L/usr/lib/X11 -lGraphicsMagick -ljbig -lwebp -llcms2 -ltiff -lfreetype -ljasper -ljpeg -lpng12 -lwmflite -lXext -lSM -lICE -lX11 -llzma -lbz2 -lxml2 -lz -lgomp

# OpenCV
#CFLAGS += `pkg-config opencv --cflags`

all: whycon-id-gen

whycon-id-gen: whycon-id-gen.cpp CNecklace.cpp
	$(CC) whycon-id-gen.cpp $(CFLAGS) $(LDFLAGS) -o $@ 

clean:
	rm -f $(OBJECTS) $(EXECUTABLE) 

cleanall:
	rm -f *.o 
