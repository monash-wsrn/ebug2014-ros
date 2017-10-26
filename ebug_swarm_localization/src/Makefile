#MACROS
CPP=clang++
CFLAGS=-c -Wall -std=c++11
LDFLAGS=
SOURCES=EBugData.cpp main.cpp
OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLE=erwin

# all:
# 	clang++ -std=c++11 main.cpp -o diocane

all: $(SOURCES) $(EXECUTABLE) #NEVER FORGET THE GUY'S NAME


$(EXECUTABLE): $(OBJECTS)
	$(CPP) $(LDFLAGS) $(OBJECTS) -o $@

.cpp.o:
	$(CPP) $(CFLAGS) $< -o $@

clean:
	rm *.o $(EXECUTABLEw)
