CC = g++
INCL = -Itao -I../tao/matrix -I../tao/dynamics -I../tao/utility \
	-I../saimatrix -I../tinyxml
CFLAGS = -Wall -ggdb -DASSERT=assert -DTESTING_FUNCTIONS_ON=1
FFLAGS = -Wall -O3

SOURCES = main2.cpp CRobotArchitect.cpp ../saimatrix/*.cpp \
	./osimarchitect/*.cpp ../tinyxml/*cpp
HEADERS = SRobotLinkStructures.hpp CRobotDefinition.hpp CRobotArchitect.hpp \
	../tao/matrix/TaoDeMath.h ../saimatrix/*.h ./osimarchitect/*h ../tinyxml/*h

TARGET = robotarchitect

.PHONY: all

all: 
	$(CC) $(INCL) $(CFLAGS) -o $(TARGET) $(SOURCES)

final: 
	$(CC) $(INCL) $(FFLAGS) -o $(TARGET) $(SOURCES)

clean: 
	@rm -f *.o $(TARGET) 

debug: 
	$(CC) $(CFLAGS) -pg -o $(TARGET) $(SOURCES)

