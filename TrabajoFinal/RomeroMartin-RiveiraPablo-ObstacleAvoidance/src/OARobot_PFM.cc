#include <iostream>
#include <string>
#include <stdlib.h>
#include <math.h>
#include <fstream> 
#include <gtkmm.h>
#include<numeric>
#include <chrono>
#include<algorithm>
#include <robot.hh>
#include <environment.hh>
#include <simulation.hh>
#include <simulation-plot.hh>

bool  DONE=true;
float medias=0.0;
float contador=0.0;
float minimo_absoluto=100.0;
float auxi=0.0;
float meanNonZero(const std::vector<float>& myList) {
    double sum = std::accumulate(myList.begin(),myList.end(),0.0,[](double a,double b){return b!=0 ? a+b : a;});
    double count = std::count_if(myList.begin(),myList.end(),[](double i){return i!=0;});
    return (count == 0) ? 0 : sum/count;
}
float minNonZero(const std::vector<float>& myList) {
    auto minIt = std::min_element(myList.begin(),myList.end(),[](double a,double b){return a>0 && b>0 ? a<b : a>b;} );
    int count = std::count_if(myList.begin(),myList.end(),[](double i){return i>0;});
    return (count == 0) ? 0 : *minIt;
}
// nromaliza angulos para mi intervalo de (-pi,pi]
  float norm_some_angle( float angulo){
    while (angulo>M_PI){
      angulo=angulo-2*M_PI;

    }
    while(angulo<-M_PI){
      angulo=angulo+2*M_PI;
    }
    return angulo;}

// #define INTEGRATOR


#ifdef INTEGRATOR

#else
class OARobot : public rs::Robot {
public:
  
  OARobot(const rs::Pose & p, 
	  const rs::RobotSettings & settings =
	  rs::defaultRobotSettings):
    rs::Robot(p, settings)
  {}

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

        x=m_R[ii]*cos(-M_PI+ii*delta_ang+ang_rob);
        y=m_R[ii]*sin(-M_PI+ii*delta_ang+ang_rob);
        
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
    ang_rob=norm_some_angle(ang_rob);
    alpha=norm_some_angle(alpha);
    float vel_ang=K*norm_some_angle(alpha-ang_rob)*sigmoide;
    float vel_lin=sigmoide*ponderacion2;
    //posible implementacion basada en pr1
    //float aux=(-ang_rob-alpha);
    //float kb=-1.5;
    //float ka=abs(2.5*sigmoide);

    //float sigmoide2=1/(1+exp(-aux));
    //float vel_ang=(kb*aux+ka*alpha);
 
    // std::cout<<x_rob<<std::endl;
      // std::cout<<"++++++++++++++++++++++++++++++++++++++"<<std::endl;
      // std::cout<<"final"<<std::endl;
      // std::cout<<vel_lin<<std::endl;
      // std::cout<<vel_ang<<std::endl;
      // std::cout<<"++++++++++++++++++++++++++++++++++++++"<<std::endl;   
    m_vel=rs::Velocity(vel_lin,vel_ang);

    if (x_rob<0.001 && y_rob<0.001) {
      DONE=true;
    }
    else{
      DONE=false;
    }
    

    medias=medias+meanNonZero(m_R);
    contador=contador+1;

    
    auxi=minNonZero(m_R);
    if (auxi<minimo_absoluto){
      minimo_absoluto=auxi;
    }

    return m_vel;
  }

};
#endif

typedef std::shared_ptr<OARobot> OARobotPtr;

int
main(int argc, char *argv[])
{
  std::cout << "---OA---" << std::endl;
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

      int var=atoi(argv[3]);
      if (argc > 3){srand(var);}
      std::cout<<var<<std::endl;
      float radius=0.1;
      envp->push_back(std::make_shared<rs::Circle>(rs::Position(rand()% 5+1 ,rand()% 5+1), radius*5));
      envp->push_back(std::make_shared<rs::Circle>(rs::Position(-rand()% 5-1, rand()% 5+1), radius*5));
      envp->push_back(std::make_shared<rs::Circle>(rs::Position(rand()% 5+1, -rand()% 5-1), radius*5));
      envp->push_back(std::make_shared<rs::Circle>(rs::Position(-rand()% 5-1, -rand()% 5-1), radius*5));

      envp->push_back(std::make_shared<rs::Circle>(rs::Position(rand()% 5+1, -rand()% 5-1), radius*5));
      envp->push_back(std::make_shared<rs::Circle>(rs::Position(-rand()% 5-1, -rand()% 5-1), radius*5));

                  // for (unsigned int ii = 0; ii < envp->size(); ii++)
                  //     if ((*envp)[ii]->isIn(rs::Position(0,0), 0)) {
                  //       std::cout <<"[INFO] Obstacles too close to target:("<< std::endl;
                  //         x=-x;y=-y;
                        
                  //     }



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
    std::make_shared<OARobot>(rs::Position(0,5),
            rSettings);
#else
float x,y,t;
    if (argc > 3){x=(rand() % 11) - 5;y=(rand() % 11) - 5;t=(rand() / (double) RAND_MAX) * 2*M_PI - M_PI;
                  std::cout <<x<<","<<y<<","<<t<< std::endl;
                  
                  for (unsigned int ii = 0; ii < envp->size(); ii++)
                      if ((*envp)[ii]->isIn(rs::Position(x,y), 0)) {
                        std::cout <<"[INFO] Initial Pose is IN an obstacle...bad luck. Changing rnd pose"<< std::endl;
                          x=-x;y=-y;
                        
                      }
                  
                  }
  else{x=3,y=0.7,t=M_PI;
  
                    for (unsigned int ii = 0; ii < envp->size(); ii++)
                      if ((*envp)[ii]->isIn(rs::Position(x,y), 0)) {
                        std::cout <<"[INFO] Initial Pose is IN an obstacle...bad luck. Changing rnd pose"<< std::endl;
                          x=-x;y=-y;
                        
                      }
  
  }
  OARobotPtr rp =
    std::make_shared<OARobot>(rs::Pose(x,y,t),
            rSettings);

// else{
//   OARobotPtr rp =
//     std::make_shared<OARobot>(rs::Pose(5,0,-M_PI),
//             rSettings);

// }
#endif
 
 
  // Create a Simulation object to simulate the swarm
  rs::SimulationPtr simp(std::make_shared<rs::Simulation>(envp));

  // Run the simulation
  //simp->run(std::dynamic_pointer_cast<OARobotIntegrator>(rp));
  rs::RobotPtr rp2 = std::dynamic_pointer_cast<rs::Robot>(rp);

  auto start = std::chrono::high_resolution_clock::now();
  simp->run(rp2);
  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
 
  // Uncomment this if you want to inspect the trajectory as a text
  // file (see header file simulation.hh to check for the format)
  simp->saveTraj(filename.c_str());
 

  std::fstream fich("datos/_PFM.dat",  std::fstream::in |  std::fstream::out |  std::fstream::app);

  if (!fich)
  {
    std::cout << "Error al abrir datos/_PFM.dat\n";
    exit(EXIT_FAILURE);
  }
  fich << DONE << "," <<minimo_absoluto<< "," <<medias/contador<< "," <<duration<< std::endl  ;
    fich.close();










  // Show the animation of the retulting swarm trajectory

  if (argc>4 and atoi(argv[4])){
  Glib::RefPtr<Gtk::Application>  app = Gtk::Application::create("es.usc.rs");
  rs::SimulationWindow dsp(simp);
  app->run(dsp);
  }  return 0;
}

