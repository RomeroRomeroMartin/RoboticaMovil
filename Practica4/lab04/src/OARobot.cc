#include <iostream>
#include <string>
#include <stdlib.h>
#include <math.h>
#include <math.h>
#include <gtkmm.h>


#include <robot.hh>
#include <environment.hh>
#include <simulation.hh>
#include <simulation-plot.hh>

float normSomeThing(float val, float min,float max) {
   if (val >= min) {
     return min + (val - min) /(max - min);
   } else {
     return max - (min - val) / (max -min);
   }
}

float normRad(float val) {
  return normSomeThing(val, -M_PI, M_PI);
}
//#define INTEGRATOR
#define pi 3.14159265

#ifdef INTEGRATOR

class OARobot : public rs::Robot {
public:
  OARobot(const rs::Position & p, 
	  const rs::RobotSettings & settings =
	  rs::defaultRobotSettings):
    rs::Robot(p, settings)
  {
  }
  
  const rs::Velocity & action()
  {
    // TODO: Implement a PFM for the integrator robot
    std::vector<float>f(2);
    float x_rob=m_p.pos[0];
    float y_rob=m_p.pos[1];
    float x,y,x_vec,y_vec,x_des,y_des;
    float ponderacion=1;
    float delta_ang=360/m_R.size();
    for (unsigned int ii=0;ii<m_R.size();ii++){
      if (m_R[ii]<0.9){ 
        x=m_R[ii]*cos(delta_ang*ii*pi/180);
        y=m_R[ii]*sin(delta_ang*ii*pi/180);
        
        
        x_vec=x_rob-x;
        y_vec=y_rob-y;
        float ponderacion=1/m_R[ii];

        f[0]=f[0]+(1/(sqrt(pow(x_vec,2)+pow(y_vec,2))));
        f[1]=f[1]+(1/(sqrt(pow(x_vec,2)+pow(y_vec,2))));
    
    
      }
      }
    float ponderacion2=sqrt(abs(pow(0-x_rob,2)+pow(0-y_rob,2)));
    x_des=0-x_rob/(sqrt(pow(x_rob,2)+pow(y_rob,2)));
    y_des=0-y_rob/(sqrt(pow(x_rob,2)+pow(y_rob,2)));

    float sigmoide=1/(1+exp(-ponderacion2));

    m_vel=rs::Velocity(sigmoide*(ponderacion*f[0]+ponderacion2*x_des),sigmoide*(ponderacion*f[1]+ponderacion2*y_des));
    return m_vel; 
  }

};
#else
class OARobot : public rs::Robot {
public:
  
  OARobot(const rs::Pose & p, 
	  const rs::RobotSettings & settings =
	  rs::defaultRobotSettings):
    rs::Robot(p, settings)
  {}
  //float normaliza(float angulo);
  float normaliza( float angulo){
    while (angulo>M_PI){
      angulo=angulo-2*pi;

    }
    while(angulo<-pi){
      angulo=angulo+2*pi;
    }
    return angulo;}
  const rs::Velocity & action() 
  {
    // TODO: Implement a PFM for the unicycle robot   
    std::vector<float>f(2);
    std::vector<float>f_final(2);
    float x_rob=m_p.pose[0];
    float y_rob=m_p.pose[1];
    float ang_rob=m_p.pose[2];
    float x,y,x_vec,y_vec,x_des,y_des;
    float ponderacion=1;
    float alpha;
    std::vector<float>eje(2);
    eje[0]=1;
    eje[1]=0;
    std::vector<float>eje2(2);
    eje2[0]=1;
    eje2[1]=0;
    float delta_ang=360/m_R.size();
    for (unsigned int ii=0;ii<m_R.size();ii++){
      if (m_R[ii]<0.9){ 

        x=m_R[ii]*cos(-pi+ii*delta_ang+ang_rob);
        y=m_R[ii]*sin(-pi+ii*delta_ang+ang_rob);
        
        x_vec=x_rob-x;
        y_vec=y_rob-y;

        float ponderacion=1/m_R[ii];

        f[0]=f[0]+ponderacion*(x_vec/(sqrt(pow(x_vec,2)+pow(y_vec,2))));
        f[1]=f[1]+ponderacion*(y_vec/(sqrt(pow(x_vec,2)+pow(y_vec,2))));

        
      }

    } 
    float ponderacion2=sqrt(pow(0-x_rob,2)+pow(0-y_rob,2));
    x_des=0-x_rob/(sqrt(pow(x_rob,2)+pow(y_rob,2)));
    y_des=0-y_rob/(sqrt(pow(x_rob,2)+pow(y_rob,2)));
    float sigmoide=1/(1+exp(-ponderacion2));

    f_final[0]=ponderacion*f[0]+x_des;
    f_final[1]=ponderacion*f[1]+y_des;
   
    //float t = ((eje[0])*(f_final[0]) + (eje[1])*(f_final[1]))/ (sqrt(pow(eje[0], 2) + pow(eje[1], 2))*sqrt(pow(f_final[0], 2) + pow(f_final[1], 2)));
    //alpha=acos(t);
    alpha=(atan2(f_final[1],f_final[0]));
    float K=1.8;
    ang_rob=normaliza(ang_rob);
    alpha=normaliza(alpha);
    float vel_ang=K*normaliza(alpha-ang_rob)*sigmoide;
    float vel_lin=sigmoide*ponderacion2;
    //float aux=(-ang_rob-alpha);
    //float kb=-1.5;
    //float ka=abs(2.5*sigmoide);

    //float sigmoide2=1/(1+exp(-aux));
    //float vel_ang=(kb*aux+ka*alpha);
    //vel_lin=0;
    std::cout<<x_rob<<std::endl;
    m_vel=rs::Velocity(vel_lin,vel_ang);
    return m_vel;
  }

};
#endif

typedef std::shared_ptr<OARobot> OARobotPtr;

int
main(int argc, char *argv[])
{
  std::string filename = "trajectory.dat";
  if (argc > 1)
    filename = std::string(argv[1]);

  // Create an environment for the swarm
  rs::EnvironmentPtr envp =
    std::make_shared<rs::Environment>(rs::Position(-5.5,-5.5),
				      rs::Position(5.5,5.5));
  unsigned int map(0);
  if (argc > 2)
    map = atoi(argv[2]);
  
  switch (map) {
  case 0:
    {
      // Add random obstacles in the environment
      envp->push_back(std::make_shared<rs::Circle>(rs::Position(3, 0.7), 0.5));
      envp->push_back(std::make_shared<rs::Circle>(rs::Position(3, -0.7), 0.5));
      envp->push_back(std::make_shared<rs::Circle>(rs::Position(1.2, 3.5), 0.45));
      envp->push_back(std::make_shared<rs::Circle>(rs::Position(-4, -0.7), 0.5));
      envp->push_back(std::make_shared<rs::Circle>(rs::Position(2,-4), 0.2));
      envp->push_back(std::make_shared<rs::Circle>(rs::Position(-1.5,2.75), 0.3));
    }
    break;
  case 1:
    {
      rs::Position p0(-0.9,-2);
      rs::Position p1(-1.1,-2);
      rs::Position p2(-1.1,-0.9);
      rs::Position p3(1.1,-0.9);
      rs::Position p4(1.1,-2);
      rs::Position p5(0.9,-2);
      rs::Position p6(0.9,-1.1);
      rs::Position p7(-0.9,-1.1);
      std::vector<rs::Position> Pts;
      Pts.push_back(p0);
      Pts.push_back(p1);
      Pts.push_back(p2);
      Pts.push_back(p3);
      Pts.push_back(p4);
      Pts.push_back(p5);
      Pts.push_back(p6);
      Pts.push_back(p7);
      rs::PolygonPtr Pp = std::make_shared<rs::Polygon>(Pts);
      envp->push_back(Pp);
    }
    break;
  default:
    break;
  }
  
  rs::RobotSettings rSettings = {0.15,     // Robot radius
				 2.0,      // Perceptual range
				 360,      // Laser beams
				 0.0,      // Min speed
				 0.75,     // Max speed
				 0.0,      // Min Omega
				 1.5,      // Max Omega
				 {1,0,0}}; // Colour
  if (map == 1)
    rSettings.beams = 90;
  // Create a robot which avoids obstacles
#ifdef INTEGRATOR
  OARobotPtr rp =
    std::make_shared<OARobot>(rs::Position(1,-3),
			      rSettings);
#else
  OARobotPtr rp =
    std::make_shared<OARobot>(rs::Pose(0,-3, M_PI),
			      rSettings);
#endif
  
  
  // Create a Simulation object to simulate the swarm
  rs::SimulationPtr simp(std::make_shared<rs::Simulation>(envp));

  // Run the simulation
  //simp->run(std::dynamic_pointer_cast<OARobotIntegrator>(rp));
  rs::RobotPtr rp2 = std::dynamic_pointer_cast<rs::Robot>(rp);
  simp->run(rp2);
  
  // Uncomment this if you want to inspect the trajectory as a text
  // file (see header file simulation.hh to check for the format)
  simp->saveTraj(filename.c_str());
  
  // Show the animation of the retulting swarm trajectory
  Glib::RefPtr<Gtk::Application>  app = Gtk::Application::create("es.usc.rs");
  rs::SimulationWindow dsp(simp);
  app->run(dsp);
  
  return 0;
}
