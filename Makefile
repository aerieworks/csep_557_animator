#
# General Makefile for Graphics projects, last changed by dyhsiao 2014. 
#

LOCAL = /usr/local
INCLUDE = -I$(LOCAL)/include -I/usr/X11R6/include -I/opt/X11/include
LIBDIR = -L$(LOCAL)/lib -L/usr/X11R6/lib -L/opt/X11/lib

LIBS = -lfltk -lfltk_gl -lXext -lX11 -lm  -lfltk_images -lpthread -framework OpenGL -framework Cocoa -framework AGL

CFLAGS = -g #-w

CFLAGS += -MMD
  
CC = g++

OBJ_DIR = obj

SOURCE_DIR = .

TARGET = animator

CPP_FILES := $(notdir $(shell find $(SOURCE_DIR) -name '*.cpp'))
OBJ_FILES += $(addprefix $(OBJ_DIR)/, $(CPP_FILES:.cpp=.o))

CXX_FILES := $(notdir $(shell find $(SOURCE_DIR) -name '*.cxx'))
OBJ_FILES += $(addprefix $(OBJ_DIR)/, $(CXX_FILES:.cxx=.o))

PATHS = $(sort $(dir $(shell find $(SOURCE_DIR) -name '*.c*')))

DEP = $(OBJ_FILES:.o=.d)
-include $(DEP)

.SUFFIXES: .o .cpp .cxx

vpath %.cpp $(PATHS)
vpath %.cxx $(PATHS)

$(TARGET): $(OBJ_FILES)
	$(CC) -o $@ $(INCLUDE) $(LIBS) $(LIBDIR) $^

$(OBJ_FILES): | $(OBJ_DIR)

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

$(OBJ_DIR)/%.o: %.cpp
	@echo Compiling $<
	$(CC) $(CFLAGS) -c -o $@ $(INCLUDE) $<

$(OBJ_DIR)/%.o: %.cxx
	@echo Compiling $<
	$(CC) $(CFLAGS) -c -o $@ $(INCLUDE) $<

clean:
	rm -f $(TARGET) $(OBJ_DIR)/*.*
	rmdir $(OBJ_DIR)
