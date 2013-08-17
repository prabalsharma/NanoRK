SOX_ROOT=
SOX_LIBRARY_PATH=

include $(SOX_LIBRARY_PATH)/vars.mk

CC=gcc
LIBS_SOX_SAGA = -l$(SOX_LIB_NAME) -lstrophe -lexpat -lcurl -lsqlite3 -lm
LIBS_SAGA = -lexpat -lcurl -lsqlite3 -lm
LIBS_SOX = -l$(SOX_LIB_NAME) -lstrophe -lexpat -lcurl
LIBS = -lexpat -lsqlite3 -lm
INCLUDE_SOX=-I. -I../common/ -I$(STROPHE_INCLUDE) -I$(STROPHE_INCLUDE_SRC) -I$(SOX_LIBRARY_PATH)
INCLUDE=-I.
CFLAGS= -g -Wall $(INCLUDE)
CFLAGS_SOX= -g -Wall $(INCLUDE_SOX)
LDFLAGS=-L. $(LIBS)
LDFLAGS_SOX_SAGA=-L. -L$(SOX_LIBRARY_PATH) -L$(STROPHE_LIB) $(LIBS_SOX_SAGA)
LDFLAGS_SAGA=-L. -L$(SOX_LIBRARY_PATH) -L$(STROPHE_LIB) $(LIBS_SAGA)
LDFLAGS_SOX=-L. -L$(SOX_LIBRARY_PATH) -L$(STROPHE_LIB) $(LIBS_SOX)

all: slipclient.o slipstream.o pkt.o ffdb.o
	gcc -o slip_gateway slipclient.o slipstream.o pkt.o ffdb.o $(CFLAGS_SOX) $(LDFLAGS_SOX_SAGA)
	
saga_sox: slipclient_saga_sox.o slipstream.o pkt.o ffdb.o
	gcc -o slip_gateway slipclient_saga_sox.o slipstream.o pkt.o ffdb.o $(CFLAGS_SOX) $(LDFLAGS_SOX_SAGA)	
	
sox: slipclient_sox.o slipstream.o pkt.o
	gcc -o slip_gateway slipclient_sox.o slipstream.o pkt.o $(CFLAGS_SOX) $(LDFLAGS_SOX)

saga: slipclient_saga.o slipstream.o pkt.o ffdb.o
	gcc -o slip_gateway slipclient_saga.o slipstream.o pkt.o ffdb.o $(CFLAGS) $(LDFLAGS_SAGA)
	
slipclient.o: slipclient.c
	gcc -c -o $@ $< $(CFLAGS) $(LDFLAGS)
	
slipclient_saga_sox.o: slipclient.c
	gcc -c -o $@ $< $(CFLAGS_SOX) $(LDFLAGS_SOX_SAGA)
	
slipclient_saga.o: slipclient.c
	gcc -c -o $@ $< $(CFLAGS) $(LDFLAGS_SAGA)
	
slipclient_sox.o: slipclient.c
	gcc -c -o $@ $< $(CFLAGS) $(LDFLAGS_SOX)
	
pkt.o: pkt.c
	gcc -c -o $@ $< $(CFLAGS) $(LDFLAGS)
	
slipstream.o: slipstream.c
	gcc -c -o $@ $< $(CFLAGS) $(LDFLAGS)

ffdb.o: ffdb.c
	gcc -c -o $@ $< $(CFLAGS) $(LDFLAGS_SAGA)	

clean: 
	rm -f *.o *~ core slip_gateway 

 
