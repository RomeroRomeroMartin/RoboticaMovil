GXX = g++ -std=c++11
CXXFLAGS = `pkg-config --cflags eigen3` `pkg-config --cflags gtkmm-3.0` #-Wall 
LDFLAGS = `pkg-config --libs gtkmm-3.0`
AR = ar 

SRC = simulation-plot.cc simulation.cc environment.cc robot.cc 

OBJ = $(SRC:.cc=.o)

%.o: %.cc %.hh
	$(GXX) $(CXXFLAGS) -o $@ -c $<


all: libRS.a OARobot_PFM   OARobot_CVM  OARobot_VFH 
 

OARobot_VFH: OARobot_VFH.o
	$(GXX) -o $@ $< -L. -lRS $(LDFLAGS)

OARobot_VFH.o: OARobot_VFH.cc
	$(GXX) $(CXXFLAGS) -I. -o $@ -c $<



OARobot_CVM: OARobot_CVM.o
	$(GXX) -o $@ $< -L. -lRS $(LDFLAGS)

OARobot_CVM.o: OARobot_CVM.cc
	$(GXX) $(CXXFLAGS) -I. -o $@ -c $<


OARobot_PFM: OARobot_PFM.o
	$(GXX) -o $@ $< -L. -lRS $(LDFLAGS)

OARobot_PFM.o: OARobot_PFM.cc
	$(GXX) $(CXXFLAGS) -I. -o $@ -c $<









libRS.a: $(OBJ)
	$(AR) rcs $@ $^
	ranlib $@

	
robot.o: robot.cc robot.hh
environment.o: environment.cc environment.hh
simulation.o: simulation.cc simulation.hh
simulation-plot.o: simulation-plot.cc simulation-plot.hh

clean:
	rm *.o *~ libRS.a OARobot_PFM   OARobot_CVM OARobot_VFH 
