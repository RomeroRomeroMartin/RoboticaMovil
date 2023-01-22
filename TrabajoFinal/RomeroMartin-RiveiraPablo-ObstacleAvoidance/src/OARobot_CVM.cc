#include <iostream>
#include <string>
#include <stdlib.h>
#include <math.h>
#include <gtkmm.h>
#include <limits>
#include <algorithm>
#include <cstddef>
#include<vector>
#include <robot.hh>
#include <environment.hh>
#include <simulation.hh>
#include <simulation-plot.hh>
#include <numeric> 
#include <chrono>
#include <fstream> 

bool DONE=true;
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

struct Circle {
  double x;
  double y;
  double r;
};

Circle calculateCircle(double x1, double y1, double x2, double y2, double x3, double y3) {
  // Calculate the lengths of the three sides of the triangle formed by the three points
  double side1 = std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
  double side2 = std::sqrt((x2 - x3) * (x2 - x3) + (y2 - y3) * (y2 - y3));
  double side3 = std::sqrt((x1 - x3) * (x1 - x3) + (y1 - y3) * (y1 - y3));

  // Calculate the semiperimeter of the triangle
  double semiperimeter = (side1 + side2 + side3) / 2.0;

  // Calculate the area of the triangle using Heron's formula
  double area = std::sqrt(semiperimeter * (semiperimeter - side1) * (semiperimeter - side2) * (semiperimeter - side3));

  // Calculate the radius as the area divided by the semiperimeter
  double radius = area / semiperimeter;

  // Calculate the center of the circle
  double centerX = (x1 + x2 + x3) / 3.0;
  double centerY = (y1 + y2 + y3) / 3.0;

  Circle circle = {centerX, centerY, radius};
  return circle;
}

float findMinimum(const std::vector<float>& list) {
  // Use the std::min_element function to find the minimum value in the list
  return *std::min_element(list.begin(), list.end());
}

std::size_t findPosition(float value, const std::vector<float>& array) {
  // Use the std::find function to find the position of the value in the array
  std::vector<float>::const_iterator pos = std::find(array.begin(), array.end(), value);
  if (pos == array.end()) {
    // The value was not found in the array
    return array.size();
  } else {
    // The value was found in the array, so return its position
    return pos - array.begin();
  }
}

std::vector<float> curvature_velocity(float x, float y, float theta,float x_obs,float y_obs,float r_obs)
{
  constexpr float MAX_CURVATURE = 1.0;
  constexpr float MAX_VELOCITY = 1.0;

  // Calculate the distance between the unicycle and the obstacle
  float d = std::sqrt((x - x_obs) * (x - x_obs) + (y - y_obs) * (y - y_obs));
  
  // Check if the unicycle is within the obstacle's radius
  if (d < r_obs)
  {
    // If the unicycle is inside the obstacle's radius, we need to move away from the obstacle.
    // Calculate the direction to move away from the obstacle.
    float alpha = std::atan2(y_obs - y, x_obs - x);
    // Calculate the curvature command to turn the unicycle towards the direction to move away from the obstacle.
    float curvature = std::tan(alpha - theta);
    // Clamp the curvature command to the maximum curvature
    if (curvature < -MAX_CURVATURE)
      curvature = -MAX_CURVATURE;
    else if (curvature > MAX_CURVATURE)
      curvature = MAX_CURVATURE;
    // Set the velocity command to move away from the obstacle at maximum speed.
    float velocity = MAX_VELOCITY;
    // Return the curvature and velocity commands as a vector
    return {curvature, velocity};
  }
  else
  {
    // If the unicycle is outside the obstacle's radius, we can continue on the current path.
    // Return the curvature and velocity commands as a vector with zero curvature and velocity.
    return {0.0, 0.0};
  }
}

//#define INTEGRATOR
#define pi 3.14159265

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
    //float ponderacion=1;
    float alpha;
    std::vector<float>eje(2);
    eje[0]=1;
    eje[1]=0;
    std::vector<float>eje2(2);
    std::vector<float>vecFront(60);
    eje2[0]=1;
    eje2[1]=0;
    float delta_ang=360/m_R.size();
    
    int aux=0;
    for (int i=0;i<m_R.size();i++){
      if (i>150 and i<211){ 
        vecFront[aux]=m_R[i];
        aux=aux+1;
    }}
    float position2=0;
    float position3=0;
    float min2=0;
    float min3=0;
    float va=0;
    float vl=0;
    float min=findMinimum(vecFront);
    float minAbs=findMinimum(m_R);
    std::vector<float>vel_cvm(2);

    std::size_t position = findPosition(min, m_R);
    

    float ponderacion2=sqrt(pow(0-x_rob,2)+pow(0-y_rob,2));

    x_des=0-x_rob/(sqrt(pow(x_rob,2)+pow(y_rob,2)));
    y_des=0-y_rob/(sqrt(pow(x_rob,2)+pow(y_rob,2)));
    float sigmoide=1/(1+exp(-ponderacion2));


    //std::cout<<min<<std::endl;
    if (min<0.9 and min!=0){ 
      position2=position-1;
      position3=position+1;
      if (position2<0){ 
        position2=360;
      }
      if (position3>360){
        position3=0;
      }
        
      min2=m_R[position2];
      min3=m_R[position3];

      float x1=min*cos(-pi+position*delta_ang+ang_rob);
      float y1=min*sin(-pi+position*delta_ang+ang_rob);

      float x2=min2*cos(-pi+position2*delta_ang+ang_rob);
      float y2=min2*sin(-pi+position2*delta_ang+ang_rob);

      float x3=min3*cos(-pi+position3*delta_ang+ang_rob);
      float y3=min3*sin(-pi+position3*delta_ang+ang_rob);

      Circle circle = calculateCircle(x1, y1, x2, y2, x3, y3);

      std::vector<float>vel_cvm=curvature_velocity(x_rob,y_rob,ang_rob,circle.x,circle.y,circle.r);
      x_des=0;
      y_des=0;
    }
    
    f_final[0]=vel_cvm[0]+x_des;
    f_final[1]=vel_cvm[1]+y_des;
   

    alpha=(atan2(f_final[1],f_final[0]));
    float K=1.8;
    ang_rob=normaliza(ang_rob);
    alpha=normaliza(alpha);
    float vel_ang=K*normaliza(alpha-ang_rob)*sigmoide;
    float vel_lin=sigmoide*ponderacion2;

    // std::cout<<x_rob<<std::endl;
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

typedef std::shared_ptr<OARobot> OARobotPtr;

int
main(int argc, char *argv[])
{
    std::cout << "---CVM---" << std::endl;
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
float x,y,t;
    if (argc > 3){x=(rand() % 11) - 5;y=(rand() % 11) - 5;t=(rand() / (double) RAND_MAX) * 2*M_PI - M_PI;
                  std::cout <<x<<","<<y<<","<<t<< std::endl;
                  
                  for (unsigned int ii = 0; ii < envp->size(); ii++)
                      if ((*envp)[ii]->isIn(rs::Position(x,y), 0)) {
                        std::cout <<"[INFO] Initial Pose is IN an obstacle...bad luck. Changing rnd pose"<< std::endl;
                          x=-x;y=-y;
                        
                      }
                  
                  }
  else{x=5,y=0,t=M_PI;}
  OARobotPtr rp =
    std::make_shared<OARobot>(rs::Pose(x,y,t),
            rSettings);    
  OARobotPtr rp3 =
    std::make_shared<OARobot>(rs::Pose(5,-1,-M_PI),
			      rSettings);
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
  

  std::fstream fich("datos/_CVM.dat",  std::fstream::in |  std::fstream::out |  std::fstream::app);

  if (!fich)
  {
    std::cout << "Error al abrir datosCVM.dat\n";
    exit(EXIT_FAILURE);
  }
  fich << DONE << "," <<minimo_absoluto<< "," <<medias/contador<< "," <<duration<< std::endl  ;
    fich.close();


  // Show the animation of the retulting swarm trajectory
  
  if (argc>4 and atoi(argv[4])){
  Glib::RefPtr<Gtk::Application>  app = Gtk::Application::create("es.usc.rs");
  rs::SimulationWindow dsp(simp);
  app->run(dsp);
  }


/*
  // Create a Simulation object to simulate the swarm
  rs::SimulationPtr simp2(std::make_shared<rs::Simulation>(envp));

  // Run the simulation
  //simp->run(std::dynamic_pointer_cast<OARobotIntegrator>(rp));
  rs::RobotPtr rp4 = std::dynamic_pointer_cast<rs::Robot>(rp3);
  auto start2 = std::chrono::high_resolution_clock::now();
  simp2->run(rp4);
  auto end2 = std::chrono::high_resolution_clock::now();
  auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(end2 - start2).count();
  
  // Uncomment this if you want to inspect the trajectory as a text
  // file (see header file simulation.hh to check for the format)
  simp2->saveTraj(filename.c_str());
  
  std::ofstream fich2("datosCVM2.dat");
  if (!fich2)
  {
    std::cout << "Error al abrir datosCVM2.dat\n";
    exit(EXIT_FAILURE);
  }
  fich2 << DONE << "," <<minimo_absoluto<< "," <<medias/contador<< "," <<duration2 <<std::endl;
  // Show the animation of the retulting swarm trajectory
  Glib::RefPtr<Gtk::Application>  app2 = Gtk::Application::create("es.usc.rs");
  rs::SimulationWindow dsp2(simp2);
  app2->run(dsp2);*/
  return 0;
}