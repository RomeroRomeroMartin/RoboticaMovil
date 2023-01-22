/*
 * simulation.cc (C) Inaki Rano 2022
 * 
 * Definition of the simulation class for the 2D robot simulator
 *
 *
 *
 */
#include <fstream>
#include <yaml-cpp/yaml.h>

#include "simulation.hh"

namespace rs {

    
  void
  Simulation::run(RobotPtr & r)
  {
    
    float t(0);
    m_traj.push_back(r->pose());
    while (t < m_tMax) {
      r->scan(m_envp);
      r->action();
      r->step(m_dt);
      m_traj.push_back(r->pose());
      m_scans.push_back(r->scan());
      t += m_dt;
      // Check if the robot crashed
      Position p(r->position());
      for (unsigned int ii = 0; ii < m_envp->size(); ii++)
	if ((*m_envp)[ii]->isIn(p, 0)) {
	  m_tMax = t;
	  std::cout << "Robot Crashed!!" << std::endl;
	}
    }
    
    return;
  }
  

  void
  Simulation::saveTraj(const char * filename) const
  {
    std::ofstream ofd(filename);
    for (unsigned int ii = 0; ii < m_traj.size(); ii++)
      {
	ofd << ii * m_dt << " " << m_traj[ii].transpose()[0] <<" "<< m_traj[ii].transpose()[1]<<" "<< m_traj[ii].transpose()[2];
	for (unsigned jj = 0; jj < m_scans[ii].size(); jj++)
	  ofd << m_scans[ii][jj] << " ";
	ofd <<std::endl;
      }
    
    ofd.close();
  }
  
}
