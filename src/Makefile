#Folders
HEADERS = ../headers 
SOURCES = .
OBJECTS = ../obj
BUILD = ../build
SHADERS = ../shaders
LIBPATH = ../libs

#Compiler details
CC=g++
CFLAGS = -Wall -g -I$(HEADERS) #warning, debugging, compile 
 
#Files
SOURCE_FILES = $(SOURCES)/main.cpp $(SOURCES)/functions.cpp $(SOURCES)/matrix.cpp
OBJECT_FILES = $(OBJECTS)/main.o $(OBJECTS)/functions.o $(OBJECTS)/matrix.o
TARGET = ../build/demo

#Libraries
EXTRA_LIBS = $(LIBPATH)/libGLEW.a $(LIBPATH)/libglfw3.a
LIBS = -lGL -lX11 -lXxf86vm -lXrandr -lpthread -lXi -lm -lXinerama -lXcursor

all: $(TARGET)

$(TARGET): $(OBJECT_FILES)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJECT_FILES) $(EXTRA_LIBS) -I include $(LIBS)

$(OBJECTS)/main.o: $(SOURCES)/main.cpp 
	$(CC) $(CFLAGS) -c $(SOURCES)/main.cpp -o $(OBJECTS)/main.o 

$(OBJECTS)/functions.o: $(SOURCES)/functions.cpp 
	$(CC) $(CFLAGS) -c $(SOURCES)/functions.cpp -o $(OBJECTS)/functions.o 

$(OBJECTS)/matrix.o: $(SOURCES)/matrix.cpp 
	$(CC) $(CFLAGS) -c $(SOURCES)/matrix.cpp -o $(OBJECTS)/matrix.o 

clean:
	rm -f $(OBJECT_FILES) $(TARGET)

