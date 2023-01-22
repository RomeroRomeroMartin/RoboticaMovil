/*
 * simulation.hh (C) Inaki Rano 2022
 * 
 * Declaration of environment class for the 2D robot simulator
 *
 * The simulation class performs the simulation of a swarm given the
 * swarm, a simulation time (100m by default) and a step time (0.1s by
 * default). It also generates random swarms of a given size given a
 * robot which is cloned as meny times as the size and set in random
 * positions in the environment. The simulation class stores the
 * swarms at every timestep of the simulation, and it can be recovered
 * using the operator [] with an index (step number) or using the
 * simulation time (a float). The number of steps in the simulation
 * can be obtained by calling the size() method.
 *
 */
#ifndef RS_SIMULATION_HH
#define RS_SIMULATION_HH
#include <string>
#include <vector>

#include "environment.hh"
#include "robot.hh"


namespace rs {
  // Forward declaration of the Simulation class.
  class Simulation;

  // Shared pointer to a simulation object type definition, needs to
  // be used when displaying the simulation
  typedef std::shared_ptr<Simulation> SimulationPtr;

  // Simulation class definition
  class Simulation {
  public:
    // Constructor requires the swarm to simulate
    Simulation(const EnvironmentPtr & envp): m_dt(0.1), m_tMax(90)
    { m_envp = envp; }

    // get and set time increment for the simulation used in the Euler
    // integration method. The time is propagated to the swarm and
    // from it to the individual robots which actually perform their
    // integration step.
    float dt(void) const { return m_dt; }
    void dt(float dt) { m_dt = dt; }

    // Maximum simulation time, all simulations start at t=0 and run
    // until t=tmax()
    float tMax(void) const { return m_tMax; }
    void tMax(float tMax) { m_tMax = tMax; }

    const EnvironmentPtr & environment() const { return m_envp; }
    
    // Run the simulation of the swarm. This function must be called
    // only once and no checks are done. The trajectory of the swarm
    // (the swarms at each time step) is stored internally by the
    // class.
    void run(RobotPtr &);

    // Return the size of the simulation, i.e. the number of steps
    // stored.
    unsigned int size() const { return m_traj.size(); }

    // Returns the state of the swarm at step idx as a shared pointer
    // to a swarm, this represents the state at time t=dt*idx. If the
    // index is larger than the size it returns the last swarm.
    const Pose & operator[] (unsigned int idx) const
    { return (idx < size() ? m_traj[idx] : m_traj[size() - 1]); }

    // Returns the state of the swarm at time 't' as a shared pointer
    // to a swarm. If the time is outside the range, i.e. t<0 or
    // t>tMax. it returns a null shared pointer
    const Pose operator[] (float t) const
    {
      if (t < 0)
	return m_traj[0];
      else
	if (t > m_tMax)
	  return m_traj[size() - 1];
      return m_traj[int(floor(t / m_dt))];
    }

    const std::vector<float> & scan(unsigned int idx) const
    { return (idx < size() ? m_scans[idx] : m_scans[size() - 1]); }

    // Returns the state of the swarm at time 't' as a shared pointer
    // to a swarm. If the time is outside the range, i.e. t<0 or
    // t>tMax. it returns a null shared pointer
    const std::vector<float> & scan(float t) const
    {
      if (t < 0)
	return m_scans[0];
      else
	if (t > m_tMax)
	  return m_scans[size() - 1];
      return m_scans[int(floor(t / m_dt))];
    }
    
    
    // Saves the trajectory of the swarm as a plain text file with the
    // following format:
    //
    // time  x-pos-robot y-pos-robot theta-pos-robot
    //
    // so the file has size() rows and 3 colums 
    void saveTraj(const char * ) const;

    
  private:
    // Private constructors (default and copy)
    Simulation(const Simulation & sim) {}

    EnvironmentPtr m_envp;
    float m_dt;   // simulation time step for the Euler algorithm
    float m_tMax; // Simulation time
    
    std::vector<Pose> m_traj;
    std::vector<std::vector<float> > m_scans;
  };
}
#endif
