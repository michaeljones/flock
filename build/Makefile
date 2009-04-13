OSVER := $(shell uname)
INCDIR = -I/home/mike/projects/tools/include/OpenEXR
INCDIR +=-I/usr/X11R6/include

CCFLAGS = -g -Wall -DUNIX -funroll-loops -O3 -DUNIX
CCFLAGS+=-DLINUX

LIBS= -L/home/mike/projects/tools/lib

OBJDIR = obj/
OBJECTS =  $(OBJDIR)main.o $(OBJDIR)Flock.o $(OBJDIR)Boid.o \
			$(OBJDIR)World.o $(OBJDIR)Goal.o $(OBJDIR)Object.o $(OBJDIR)Particle.o

XLIBS =  
XLIBS +=  -lGL -lGLU -lglut -lstdc++ -lImath

VPATH = ../src

LINK_TARGET = flock


all	:	$(LINK_TARGET)
$(LINK_TARGET) : $(OBJECTS)
	g++ -o $(LINK_TARGET) $(CCFLAGS) $(LIBS) $(INCDIR)  $(MATHS) \
    	   $(OBJECTS)  $(XLIBS) $(GRAPHICSLIB)

SRCDIR = ../src/

$(OBJECTS): $(OBJDIR)%.o: $(SRCDIR)%.cpp
	g++ -D$(ARCH) -c $(FLAGS) $(GRAPHICSLIB) $(CCFLAGS) $(INCDIR) $< -o $@

clean :
	rm -f $(OBJDIR)*.o; rm -f $(LINK_TARGET); 

