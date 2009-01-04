#first we find what OS were on and define OSVER to allow the different versions of the code
OSVER := $(shell uname)
# on a mac so we define DARWIN
ifeq "$(OSVER)" "Darwin"
	ARCH=_DARWIN_
endif
#Linux box so we define _LINUX_
ifeq "$(OSVER)" "Linux"
	ARCH=_LINUX_
endif
#########################################################################################
# INCDIR is the flag to set the default include paths for the system
#########################################################################################
INCDIR = -I/home/mike/projects/tools/include/OpenEXR

ifeq "$(OSVER)" "Linux"
	INCDIR+=-I/usr/X11R6/include
endif
ifeq "$(OSVER)" "Darwin"
	INCDIR+=-I/usr/local/
endif


#########################################################################################
# CCFLAGS are the compile flags for compiling the code
# -g include debug info (for use with ddd or gdb) can be removed from exe using strip
# -Wall display all warnings
# -DUNIX - GraphicsLib define to set some UNIX options
# -funroll-loops unroll static loops (i.e) if loop has hard coded values these will be
# 				 replaced with more optimised procedural sections increases code size
#				 and MAY increase speed
# -O3 turn on all optimisations
# Note :- for more options and flags type man g++ (there are a lot of arch specific options)
#########################################################################################
CCFLAGS = -g -Wall -DUNIX -funroll-loops -O3 -DUNIX
LIBS=
ifeq "$(OSVER)" "Linux"
	CCFLAGS+=-DLINUX
endif
ifeq "$(OSVER)" "Darwin"
	CCFLAGS+=-DDARWIN  -bind_at_load
endif

#########################################################################################
# LIBS add library search path to the compiler
#########################################################################################
LIBS= -L/home/mike/projects/tools/lib
ifeq "$(OSVER)" "Linux"
	LIBS +=
endif
ifeq "$(OSVER)" "Darwin"
	LIBS +=
endif

#########################################################################################
#########################################################################################
# OBJECTS should contain a list of any object files to be created. These should be the same case
# as the corresponding .cpp files for the project. For example if the program has two source files
# Main.cpp and DrawFunc.cpp the OBJECTS line should read OBJECTS= Main.0 DrawFunc.o
# if multiple lines of OBJECTS are required they should be seperated using a backslash as follows
#
# OBJECTS = Main.o (use a backslash here)
# 			DrawFunc.o  note that the last line doesn't have a (backslash here )
# Note :-  can't actually put a single backslash in a Makefile as it messes up the parsing
#		   but can put a double one to show what a single one looks like (\\) end rambling bit ;-)
#########################################################################################
OBJDIR = obj/
OBJECTS =  $(OBJDIR)main.o $(OBJDIR)Flock.o $(OBJDIR)Boid.o \
			$(OBJDIR)World.o $(OBJDIR)Goal.o $(OBJDIR)Object.o $(OBJDIR)Particle.o
#########################################################################################
# XLIBS a list of the standard libs to include in the linking process
# to add libs use -l[LIBNAME] where LIBNAME is the name of the lib minus lib[LIBNAME].so
# e.g. to add libX11.so use -lX11
#########################################################################################
XLIBS =  

ifeq "$(OSVER)" "Linux"
	XLIBS +=  -lGL -lGLU -lglut -lstdc++ -lImath
endif

VPATH = ../src

#########################################################################################
# MATHS flag to add the default maths lib (some linux distros need this)
#########################################################################################
MATHS = 
# EXENAME flag to set the output executable name for the project. This is used with the
# -o flag of the linker as well as the make clean function to remove the exe
#########################################################################################
LINK_TARGET = flock
#########################################################################################
# this line links the created OBJECT files into the exe specified from EXENAME
# and includes the relevant libraries
#########################################################################################
all	:	$(LINK_TARGET)
$(LINK_TARGET) : $(OBJECTS)
	g++ -o $(LINK_TARGET) $(CCFLAGS) $(LIBS) $(INCDIR)  $(MATHS) \
    	   $(OBJECTS)  $(XLIBS) $(GRAPHICSLIB)
#########################################################################################
# rule to build the .o files from the .cpp files
# this line will compile in turn each of the FILES from the OBJECTS variable (as a .cpp)
# into a .o file for the linking stage
#########################################################################################
SRCDIR = ../src/

$(OBJECTS): $(OBJDIR)%.o: $(SRCDIR)%.cpp
	g++ -D$(ARCH) -c $(FLAGS) $(GRAPHICSLIB) $(CCFLAGS) $(INCDIR) $< -o $@

#########################################################################################
# rule to remove and backup files, .o files and the exe for a clean build
#########################################################################################
clean :
	rm -f $(OBJDIR)*.o; rm -f $(LINK_TARGET); 
#########################################################################################
# end of Makefile
#########################################################################################

