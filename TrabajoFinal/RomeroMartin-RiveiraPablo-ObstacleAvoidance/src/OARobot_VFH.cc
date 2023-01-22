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
  // Calculate the Vector Field Histogram
  float VFH[360];
  float goal_x = 0; // x-coordinate of the goal
  float goal_y = 0; // y-coordinate of the goal
  float robot_x = m_p.pose[0]; // x-coordinate of the robot
  float robot_y = m_p.pose[1]; // y-coordinate of the robot
  float current_angle = (m_p.pose[2]); // current heading angle of the robot  
  float v_a,v_l;
  float vector_magnitude;


  // float angle_window=90;
  // float range_vision=0.5;
    float angle_window=30;
    float range_vision=0.9;
    std::vector<float>f_final(2);
    float ponderacion,y_des,x_des;
    // std::cout<<robot_y<<std::endl;
  if (abs(robot_x)>0.001 && abs(robot_y)>0.001){
  for (int i = 0; i < (sizeof(VFH)/sizeof(VFH[0])); i++) {
    float angle =  norm_some_angle(i*M_PI/180);
    float distance = m_R[i]; // distance measurement at this angle
    float vector_x = distance*cos(angle); // x-component of the vector
    float vector_y = distance*sin(angle); // y-component of the vector
    vector_magnitude = sqrt(pow(vector_x, 2) + pow(vector_y, 2)); // magnitude of the vector
    float vector_angle = (atan2(vector_y, vector_x)); // angle of the vector
    VFH[i] = (vector_magnitude*cos(norm_some_angle(vector_angle - current_angle))); // calculate the VFH value
    
    // VFH[i] = vector_magnitude*i; // calculate the VFH value
  }
  
  // Check if there are any obstacles nearby
  bool obstacle_nearby = false;
  for (int i = 0; i < m_R.size(); i++) {
    //  std::cout<<i<<" "<< VFH[i] <<std::endl;
    if (m_R[i] < range_vision) { // Increase range to 1 meter
      obstacle_nearby = true;
      float ponderacion=1/m_R[i];
      break;
    }
  }
  
  if (obstacle_nearby) {
    // Calculate the direction to turn to avoid the obstacle
    float avoid_direction = 0;
    // float DDI = norm_some_angle(atan2(goal_y - robot_y, goal_x - robot_x));
    // int index_DDI = (int)(DDI*180/M_PI); // convert to degrees    
    for (int i = 1; i < 360; i++) {

      if (VFH[i] > 0) {
        // std::cout<<"direccion ";
        // std::cout<<"2 ";
        avoid_direction = (i*M_PI/180); // convert to radians
        // std::cout<<avoid_direction<<std::endl;
        break;
      }
    }

    // Set the angular and linear velocities to turn away from the obstacle
    v_a = norm_some_angle( avoid_direction );
    
    // Don't stop, turn instead



    v_l = 0.5; // Set a low linear velocity to turn smoothly
  } else {

    // Calculate the Desired Direction Index (DDI)
    float DDI = norm_some_angle(atan2(goal_y - robot_y, goal_x - robot_x));
    int index_DDI = (int)(DDI*180/M_PI); // convert to degrees
  
    // Calculate the steering angle and speed
    float steering_angle = 0;
    float speed = 0;
    for (int i = index_DDI - angle_window; i <= index_DDI + angle_window; i++) { // Increase range to X degrees
      int index = i;
      if (index < 0) {
        index += 360;
      }
      if (index >= 360) {
        index -= 360;
      }
      if (VFH[index] > speed) {
        speed = VFH[index];
        steering_angle = DDI - current_angle;
      }
      else{
      speed=1/pow((1+exp(-sqrt(pow(0-robot_x,2)+pow(0-robot_y,2)))),2)*sqrt(pow(0-robot_x,2)+pow(0-robot_y,2));

      steering_angle = DDI - current_angle;}
    }
    // Set the angular and linear velocities
    v_a = steering_angle;
    v_l = speed;
}
  float sigm=1/pow((1+exp(-sqrt(pow(0-robot_x,2)+pow(0-robot_y,2)))),4);
  // Normalize the steering angle
  v_a = norm_some_angle(v_a);
  
  // Add a penalty for excessive turning
  if (abs(v_a) > M_PI/4) {
    v_l = v_l * 0.5; // Reduce speed if turning excessively
  }

// std::cout<<"____________"<<std::endl;    
//     std::cout<<v_l<<" , "<<v_a<<std::endl;
// std::cout<<"____________"<<std::endl; 
  // std::cout<<robot_x<<std::endl; 
  m_vel=rs::Velocity(v_l*sigm,sigm*v_a);
  // m_vel=rs::Velocity(1,1);
  // Return the calculated velocities
    if (abs(robot_x)<0.001 && abs(robot_y)<0.001) {
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
  }
  else{m_vel=rs::Velocity(0,0);DONE=true;}
  return m_vel;
}

};
#endif

typedef std::shared_ptr<OARobot> OARobotPtr;

int
main(int argc, char *argv[])
{
  std::cout << "---VFH---" << std::endl;
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
//variable = limite_inferior + rand() % (limite_superior +1 - limite_inferior) ;
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
 

  std::fstream fich("datos/_VFH.dat",  std::fstream::in |  std::fstream::out |  std::fstream::app);

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
  return 0;
}

