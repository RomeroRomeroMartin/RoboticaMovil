/*
 * robot.hh (C) Inaki Rano 2022
 * 
 * Definition of the base class Robot for the 2D multi-robot
 * simulator.  Specific implementations of a robot in multirobot
 * systems should be derived from this class.
 * 
 * The robot follows an integrator model and can perceive other robots
 * in the environment within the perception range. Specifically it can
 * perceive them as a standard vector of robots and therefore it get
 * access to all their features, such as position, size, color...
 *
 * This type of robots can be used to define static objects,
 * i.e. obstacles.
 */
#ifndef RS_ROBOT_HH
#define RS_ROBOT_HH
#include <iostream>
#include <memory>
#include <vector>
#include <string>


#include "base.hh"
#include "environment.hh"

#define RS_ROBOT_INTEGRATOR 0 
#define RS_ROBOT_UNICYCLE   1

namespace rs {

  // Forward class declaration to define the pointer to a robot (smart
  // pointer)
  class Robot;
  
  // Robot shared pointer type definition. This allows to create new
  // robots using:
  //
  // std::make_shared<Robot>([constructor params])
  //
  // For derived classes the type of object to allocate must be
  // different, but a RobotPtr can still be used
  typedef std::shared_ptr<Robot> RobotPtr;

  //! Configuration and capabilities of the robot
  // 
  //  - radius: radius of the robot for plotting and collision
  //            purposes (note collisions are not detected yet)
  //  - rMax: Perception range of the robot
  //  - vMin: Minimum speed (speeds set under this value will
  //          become zero)
  //  - vMax: Maximum velocity (speeds larger than this will be
  //          clamped)
  //  - wMin: Not used
  //  - wMax: Not used
  //  - visible: Not used
  //  - color: Color of the robot for plotting and detection purposes
  typedef struct {
    float radius; // Size
    float rMax;   // Perception range
    int beams;
    float vMin;   // Minimum speed (dead-zone)
    float vMax;   // Maximum speed
    float wMin;   // Minimum rotation speed (dead-zone)
    float wMax;   // Maximum rotation speed
    float color[3]; // Colour of the robot
  } RobotSettings;

  // Default configuration to create robots
  // - radius: 0.2 m
  // - rMax: 2.5 m
  // - vMin: 0
  // - vMax: 0.5 m/s
  // - color: [0,0,0] (black)
  extern RobotSettings defaultRobotSettings;
  
  /*! 
   * This class defines the basic interface for a robot.
   *
   * The robot follows a single integrator model in both x and y
   * coordinates. The computed velocity of the robot is stored in the
   * class together with its position.
  */
  class Robot {
  public:
    /* 
     * Robot constructor parameters
     * - p: Robot position (integrator)
     * - settings: Robot settings (defautRobotSettings by default, see
     *    above)
     */
    Robot(const Position & p, 
	  const RobotSettings & settings = defaultRobotSettings):
      m_type(RS_ROBOT_INTEGRATOR),
      m_vel(Velocity::Zero()),
      m_settings(settings),
      m_R(settings.beams, settings.rMax)
    { m_p.pos = p; }

    Robot(const Pose & p, 
	  const RobotSettings & settings = defaultRobotSettings):
      m_type(RS_ROBOT_UNICYCLE),
      m_vel(Velocity::Zero()),
      m_settings(settings),
      m_R(settings.beams, settings.rMax)
    { m_p.pose = p; }
    
    /*
     * Virtual destructor does nothing
     */
    virtual ~Robot() {}
    
    
    // Unique name for each robot type
    virtual std::string name() const { return std::string("none"); }

    
    // Get the robot settings
    const RobotSettings & settings() const {return m_settings;}    

    // Get the position of the robot
    Position position() const {
      if (m_type == RS_ROBOT_INTEGRATOR)
	return Position(m_p.pos);
      else
	return Position(m_p.pose[0], m_p.pose[1]);
    }

    Pose  pose() const {
      if (m_type == RS_ROBOT_INTEGRATOR)
	return Pose(m_p.pos[0], m_p.pos[1],0);
      else
	return Pose(m_p.pose);
    }

    const std::vector<float> & scan() { return m_R; }
    // Get the velocity of the robot (must be set by calling action()
    // method)
    const Velocity & velocity() const { return m_vel; }

    // Compute the velocity of the robot based on the list of nearby
    // robots. This method is virtual to be able to redefine it for
    // different robots in the swarm, but mit *must* set the velocity
    // in the corresponding variable of the class (m_vel) and return
    // it. *Do not* return the computed velocity without setting it or
    // the robot won't move upon calling the step() method.
    virtual const Velocity & action();

    void scan(const EnvironmentPtr & );

    float r(int idx) { return m_R[idx]; }
    
    // Simulate the motion of the robot with a step size dt
    void step(float dt);
    
    // To set the position of the robot in the Swarm class
    void position(Position p) {
      if (m_type == RS_ROBOT_INTEGRATOR)
	m_p.pos = p;
      else {
	m_p.pose[0] = p[0];
	m_p.pose[1] = p[1];
      }
    }

  protected:
    


    union m_p_t {
      m_p_t() {} 
      Pose pose;
      Position pos;
    } m_p;                          // Robot position or pose
    int m_type;                 // Robot type
    Velocity m_vel;             // Robot velocity
    RobotSettings m_settings;   // Robot settings
    std::vector<float> m_R;
  };

}


#endif
