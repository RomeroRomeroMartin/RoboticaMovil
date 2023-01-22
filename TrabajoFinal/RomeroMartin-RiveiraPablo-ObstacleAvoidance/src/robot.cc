/*
 * robot.cc (C) Inaki Rano 2022
 * 
 * Implementation of the class Robot, basic virtual class to further
 * define other robots.
 */
#include "robot.hh"


namespace rs {
  // Function to wrap the position of the robot in environments with
  // circular topology

  // Default settings of a robot
  RobotSettings defaultRobotSettings = {0.2,      // Size
					5,      // Perception range
					90,      // Number of readings per scan
					0.0,      // Min. speed
					0.5,      // Max. speed
					0.0,      // Min. w
					2 * M_PI, // Max. w
					{0.0,0.0,0.0}}; // Color (black)

  void
  Robot::scan(const EnvironmentPtr & env)
  {
    Eigen::VectorXf ang1 = Eigen::VectorXf::LinSpaced(m_settings.beams,
						      -M_PI, M_PI);
    double ang0(0);
    Position p0;
    if (m_type == RS_ROBOT_UNICYCLE)
      {
	ang0 = m_p.pose[2];
	p0[0] = m_p.pose[0];
	p0[1] = m_p.pose[1];
      }
    else {
      p0 = m_p.pos;
    }
    for (int ii = 0; ii < m_settings.beams; ii++)
      {
	Position p1(m_settings.rMax * cos(ang0 + ang1[ii]),
		    m_settings.rMax * sin(ang0 + ang1[ii]));
	p1 += p0;
	SegmentPtr sp = std::make_shared<Segment>(p0, p1);
	m_R[ii] = m_settings.rMax;
	for (int jj = 0; jj < env->size(); jj++)
	  {
	    Position p;
	    if (sp->intersection(env->operator[](jj), p))
	      {
		float d((p-p0).norm());
		if (m_R[ii] > d)
		  m_R[ii] = d;
	      }
	  }
      }
  }

  // Method to get the velocity of the robot, in a typical implementation
  const Velocity &
  Robot::action() 
  {
    m_vel = Velocity(0,0);
    return m_vel;
  }
  

  // Simulate the robot motion given the current robot velocity 
  void
  Robot::step(float dt)
  {

    switch (m_type) {
    case RS_ROBOT_INTEGRATOR:
      {
	// Calculate the norm of the velocity (speed)
	float v(m_vel.norm());
    
	// Velocity will be zero if the speed is smaller than vMin
	if (v < m_settings.vMin)
	  {
	    m_vel = Velocity::Zero();
	    return;
	  }
	// Scale velocity if the speed is larger than vMax, the velocity
	// will have speed vMax
	Velocity realV(m_vel);
	if (v > m_settings.vMax)
	  realV *= (m_settings.vMax / v);
	// Update the position of the robot according to its velocity
	// using Euler method
	m_p.pos += dt * realV;
      }
      break;
    case RS_ROBOT_UNICYCLE:
      {
	float v = (m_vel[0] < m_settings.vMax) ? m_vel[0] : m_settings.vMax ;
	float w = (fabs(m_vel[1]) < m_settings.wMax) ? fabs(m_vel[1]) : m_settings.wMax ;
	if (m_vel[1] < 0)
	  w = -w;

	v = (v > m_settings.vMin) ? v : 0;
	w = (abs(w) > m_settings.wMin) ? w : 0;
	float z(m_p.pose[2]);
	m_p.pose += dt * Pose(v * cos(z), v * sin(z), w);
      }
      break;
    default:
      std::cout << "Robot::step(); Unknow robot model" << std::endl;
    }

  }

}
