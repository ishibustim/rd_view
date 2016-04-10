CC = g++

INCLUDES = 

CCFLAGS = -Wall -O

LIBS = -lX11 -lm

.cc.o:
	$(CC) -c $(CCFLAGS) $(INCLUDES) $<

all: librdview.a rd_view

librdview.a: screen_display.o rd_display.o rd_error.o rd_parse.o rd_main.o \
	rd_refresh.o rd_object.o \
	rd_engine.o rd_enginebase.o 
	ar ruv librdview.a screen_display.o ; \
	ar ruv librdview.a rd_display.o rd_refresh.o rd_error.o rd_main.o ; \
	ar ruv librdview.a rd_parse.o rd_engine.o ; \
	ar ruv librdview.a rd_enginebase.o rd_object.o 


# Place all of the object code filenames in your renderer in this rule,
# once in the dependency section and again in the command section, i.e.
#
# rd_view: librdview.a   obj_filenames_here 
#	$(CC) $(CCFLAGS) -o rd_view obj_filenames_here librdview.a $(LIBS)

rd_view: librdview.a rd_direct.o pnm_display.o color.o edge.o point.o pointh.o vector.o rd_xform.o
	$(CC) $(CCFLAGS) -o rd_view rd_direct.o pnm_display.o color.o edge.o point.o pointh.o vector.o rd_xform.o librdview.a  $(LIBS)

clean:
	-rm *.o *.a rd_view
