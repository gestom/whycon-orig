OBJS=$(patsubst %.cpp,%.o,$(wildcard *.cpp))
OBJSC=$(patsubst %.c,%.o,$(wildcard *.c))
OBJS+=$(OBJSC)

include ../Mk/local.Mk
CXXINCLUDE+=-I./  -I../common 
CXXFLAGS+=-Wall

all: $(OBJS)

.cpp.o:
	$(CXX)  $(CXXFLAGS) $(CXXDEFINE) -c  $(CXXINCLUDE) $< 

.c.o:
	$(CC)  $(FLAGS) $(CXXDEFINE) -c  $(CXXFLAGS) $(CXXINCLUDE) $< 

clean:
	$(RM) $(OBJS)
