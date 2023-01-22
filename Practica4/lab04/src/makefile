GXX = g++ -std=c++11 
CXXFLAGS = `pkg-config --cflags eigen3` `pkg-config --cflags gtkmm-3.0` -Wall
LDFLAGS = `pkg-config --libs gtkmm-3.0`
AR = ar 

SRC = simulation-plot.cc simulation.cc environment.cc robot.cc 

OBJ = $(SRC:.cc=.o)

%.o: %.cc %.hh
	$(GXX) $(CXXFLAGS) -o $@ -c $<


all: libRS.a OARobot
 
OARobot: OARobot.o
	$(GXX) -o $@ $< -L. -lRS $(LDFLAGS)

OARobot.o: OARobot.cc
	$(GXX) $(CXXFLAGS) -I. -o $@ -c $<

libRS.a: $(OBJ)
	$(AR) rcs $@ $^
	ranlib $@

	
robot.o: robot.cc robot.hh
environment.o: environment.cc environment.hh
simulation.o: simulation.cc simulation.hh
simulation-plot.o: simulation-plot.cc simulation-plot.hh

clean:
	rm *.o *~ libRS.a OARobot
