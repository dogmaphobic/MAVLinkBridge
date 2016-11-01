# Declaration of variables
CC = g++
CCFLAGS = -w -Isource -Imavlink/common -Igcomm -Iexternal
 
# File names
EXEC = mavlinkbridge
SOURCES  = $(wildcard src/*.cc)
OBJECTS  = $(SOURCES:.cc=.o)
 
# Main target
$(EXEC): $(OBJECTS)
	$(CC) $(OBJECTS) -o $(EXEC) -lpthread
 
# To obtain object files
%.o: %.cc
	$(CC) -c $(CCFLAGS) $< -o $@
 
# To remove generated files
clean:
	rm -f $(EXEC) $(OBJECTS)