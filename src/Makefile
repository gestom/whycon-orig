include Mk/local.Mk
SUBDIRS+=common
SUBDIRS+=camera
SUBDIRS+=imageproc
SUBDIRS+=gui

OBJS=$(wildcard ../obj/*.o)
LXXLIBS+=-L/usr/local/lib
LXXLIBS+=-lpthread

COPY_ETC_CMD=../etc/copy.sh
COPY_ETC_SRC=../etc
COPY_ETC_DST=../bin

swarmcon: all obj
	$(MAKE) -C main $@; \
	$(CXX) $(CXXDEFINE) -o ../bin/$@  $(OBJS) main/$@.o $(CXXFLAGS) $(LXXLIBS) -lSDL -lSDL_ttf 

all:
	echo $(OPSYS)
	@for i in $(SUBDIRS) ;\
	do \
	echo "making" all "in $(CURRENT_DIR)/$$i..."; \
	$(MAKE) -C $$i all; \
	done

gui: all
	$(MAKE) -C gui all; \
	cp gui/*.o ../obj; \

cleangui:
	$(MAKE) -C gui clean; \
	rm -f ../obj/CGui.o

obj: all
	echo "Copy objs"
	@for i in $(SUBDIRS) ;\
	do \
	echo "coping all in $(CURRENT_DIR)/$$i..."; \
	cp $$i/*.o ../obj; \
	done

../bin/.etc:
	echo "Copy dirs from etc"
	$(COPY_ETC_CMD) $(COPY_ETC_SRC) $(COPY_ETC_DST)
	touch ../bin/.etc

cleanetc:
	rm -f ../bin/.etc	

	#cp -r ../etc/$$i ../bin; 
forceetc:
	echo "Copy dirs from etc"
	$(COPY_ETC_CMD) $(COPY_ETC_SRC) $(COPY_ETC_DST)
	touch ../bin/.etc

clean: cleangui
	echo $(OPSYS)
	@for i in $(SUBDIRS) ;\
	do \
	echo "cleaning" all "in $(CURRENT_DIR)/$$i..."; \
	$(MAKE) -C $$i clean; \
	done
	$(MAKE) -C main clean; \
	echo "cleaning all objs"
	rm -f ../obj/*.o
	echo "cleaning binaries"
	rm -f ../bin/*
