#include "ros/ros.h"
#include "std_msgs/String.h"
#include "virtuose/out_virtuose_status.h"
#include "virtuose/out_virtuose_pose.h"
#include "virtuose/out_virtuose_physical_pose.h"

#include "virtuose/in_virtuose_force.h"
#include "virtuose/virtuose_impedance.h" ////cosa e' questo?
#include "virtuose/virtuose_reset.h"
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/WrenchStamped.h>
#include <iostream>
#include <fstream>

// Storage for virtuose pose and ur5 position
float cur_pose_H[7];//pose ->virtuose
float old_pose[7];
float initial_pose_H[7];
float delta_pos_H[7];
float delta_vel_H[7];

bool enable_virtual_elong_ur5=false;//false=0 true=1
bool enable_fixed_spring_effect=false;
bool enable_curr_spring_effect=false;
bool sensor_contact=false;

float cur_position_ur5[3];//position-> ur5
float old_position_ur5[3];
float delta_pos_ur5[3],delta_pos_ur5_old[3];
float delta_vel_ur5[3];
float initial_position_ur5[3]; 



int old_H_flag=0,old_ur5_flag=0;
uint64_t pose_date;
int dt=0.01;

//
//Sping Only parameters: force limit=10; KK2=50
float error_x=0, error_y =0, error_z= 0;
float error_sensitivity=0.001;//0.01;
float force_limit=10;
float damping_coeff=0;//100;
float damping_force[3];

float fixed_spring_x,fixed_spring_y,fixed_spring_z;
float spring_force[3];
float fixed_spring_x_force,fixed_spring_y_force,fixed_spring_z_force;
float curr_spring_x_force,curr_spring_y_force, curr_spring_z_force;
float test_error;
//
float virutal_elong__ur5[3];
float virutal_elong__H[3];
float KK=0;
float KK_fix=0;//100;// for VIRUTOSE ALONE SPRING TEST set 100;
float KK_curr=0;//1000;

// Storage for virtuose_node status
uint64_t status_date;
int status_state = 0;
int status_button;

float test_x,test_y,test_z;  


// Callback for topic out_virtuose_pose
void out_virtuose_poseCB(const virtuose::out_virtuose_pose::ConstPtr& msg)
{
// Store last pose date
  pose_date = msg->header.stamp.toNSec();
  cur_pose_H[0] = msg->virtuose_pose.translation.x; //virtuose_pose  cambia solo se si impugna con decisione il manettino
  cur_pose_H[1] = msg->virtuose_pose.translation.y;
  cur_pose_H[2] = msg->virtuose_pose.translation.z;
  cur_pose_H[3] = msg->virtuose_pose.rotation.x;
  cur_pose_H[4] = msg->virtuose_pose.rotation.y;
  cur_pose_H[5] = msg->virtuose_pose.rotation.z;
  cur_pose_H[6] = msg->virtuose_pose.rotation.w;

  if (old_H_flag==0) 
  {
    for(int i=0;i<7;i++)
    {
      initial_pose_H[i]=cur_pose_H[i];
      old_pose[i]=initial_pose_H[i];
    }
    old_H_flag=1;
  } 
}

// Callback for topic out_virtuose_pose
void out_virtuose_physical_poseCB(const virtuose::out_virtuose_physical_pose::ConstPtr& msg)
{
// Store last pose date

  pose_date = msg->header.stamp.toNSec();
  cur_pose_H[0] = msg->virtuose_physical_pose.translation.x; //virtuose_physical_pose  cambia anche se NON si impugna con decisione il manettino
  cur_pose_H[1] = msg->virtuose_physical_pose.translation.y;
  cur_pose_H[2] = msg->virtuose_physical_pose.translation.z;
  cur_pose_H[3] = msg->virtuose_physical_pose.rotation.x;
  cur_pose_H[4] = msg->virtuose_physical_pose.rotation.y;
  cur_pose_H[5] = msg->virtuose_physical_pose.rotation.z;
  cur_pose_H[6] = msg->virtuose_physical_pose.rotation.w;
  if (old_H_flag==0) 
  {
    for(int i=0;i<7;i++)
    {
      initial_pose_H[i]=cur_pose_H[i];
      old_pose[i]=initial_pose_H[i];
    }
    old_H_flag=1;
  } 
}

// Callback for topic out_virtuose_status
void out_virtuose_statusCB(const virtuose::out_virtuose_status::ConstPtr& msg)
{
  // Store last status date
  status_date = msg->header.stamp.toNSec();
  status_state = msg->state;
  status_button = msg->buttons;
}

// Callback for cartesian position received from UR5
void __OnCartesianPoseUR5(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  //out_virtuose_physical_pose corresponds in ur5 (-xV=-xUR5; +yV=+zUR5; +zV=+yUR5)
  cur_position_ur5[0] = msg->pose.position.x;
  cur_position_ur5[1] = msg->pose.position.y; // must be Z
  cur_position_ur5[2] = msg->pose.position.z; // must be Y

  if(old_ur5_flag==0)
  {
    for(int i=0;i<7;i++)
    {
      initial_position_ur5[i] = cur_position_ur5[i];
      old_position_ur5[i]=initial_position_ur5[i];
    }
    old_ur5_flag=1;
  } 
  //transform it into value for force feedback
}

void __OnOPTO0Contact(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{

  //std::cout << " OPTOFORCE 0 "<< "x" << msg->wrench.force.x <<"y" << msg->wrench.force.y <<"z" << msg->wrench.force.z <<"\n";
}

void __OnOPTO1Contact(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{

  //std::cout << " OPTOFORCE 1 "<< "x" << msg->wrench.force.x <<"y" << msg->wrench.force.y <<"z" << msg->wrench.force.z <<"\n";
}

void __OnOPTO2Contact(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  //std::cout << " OPTOFORCE 2 "<< "x" << msg->wrench.force.x <<"y" << msg->wrench.force.y <<"z" << msg->wrench.force.z <<"\n";
}

void __OnOPTO3Contact(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{

  //std::cout << " OPTOFORCE 3 "<< "x" << msg->wrench.force.x <<"y" << msg->wrench.force.y <<"z" << msg->wrench.force.z <<"\n";
}


int sgn(float v)
{
  if (v < 0) return -1;
  if (v > 0) return 1;
  return 0;

}

// Main function
int main(int argc, char **argv)
{
  for(int i;i<+3;i++)
  {
    cur_position_ur5[i]=0.0;
    old_position_ur5[i]=0.0;
    delta_pos_ur5[i]=0.0;
    delta_pos_ur5_old[i]=0.0;
    delta_vel_ur5[i]=0.0;
    initial_position_ur5[i]=0.0; 
  }
  std::cout << std::fixed;
  std::cout << std::setprecision(4);
  std::ofstream myfile;
  myfile.open ("UR5data.csv");

  printf("Starting test_damping node\n");
// Init ROS library
  ros::init(argc, argv, "test_impedance"); //////cosa e' questo? 
// Create node handle
  ros::NodeHandle n;

// Connect to virtuose_node
  printf("Connecting to virtuose node\n");
  ros::Subscriber out_virtuose_status = n.subscribe<virtuose::out_virtuose_status>("out_virtuose_status", 1, out_virtuose_statusCB);

  //SWITCH THESE SUBSCIBER TO CHANGE BETWEEN VIRTUAL AND PHYSICAL_POSE : to be consistent in this file one of two MUST be commented out
  //ros::Subscriber out_virtuose_pose = n.subscribe<virtuose::out_virtuose_pose>("out_virtuose_pose", 1, out_virtuose_poseCB);
  ros::Subscriber out_virtuose_physical_pose = n.subscribe<virtuose::out_virtuose_physical_pose>("out_virtuose_physical_pose", 1, out_virtuose_physical_poseCB);


  //ros::Subscriber cartesian_position_ur5 = n.subscribe("/cartesian_pose_UR5", 5, __OnCartesianPoseUR5);
  //ros::Subscriber cartesian_position_ur5 = n.subscribe("/cartesian_pose_antiRotation_UR5", 5, __OnCartesianPoseUR5);
  //out_virtuose_physical_pose corresponds in ur5 (-xV=-xUR5; +yV=+zUR5; +zV=+yUR5)
  
  //ros::Subscriber cartesian_position_ur5 = n.subscribe("/cartesian_pose_UR5_anti", 5, __OnCartesianPoseUR5);
  ros::Subscriber cartesian_position_ur5 = n.subscribe("/cartesian_pose_antiRotation_UR5", 5, __OnCartesianPoseUR5);

  //OPTOFORCE SUBSCRIBERS
  ros::Subscriber optoForce0_feedback = n.subscribe("/optoforce_wrench_0", 5, __OnOPTO0Contact);
  ros::Subscriber optoForce1_feedback = n.subscribe("/optoforce_wrench_1", 5, __OnOPTO1Contact);
  ros::Subscriber optoForce2_feedback = n.subscribe("/optoforce_wrench_2", 5, __OnOPTO2Contact);
  ros::Subscriber optoForce3_feedback = n.subscribe("/optoforce_wrench_3", 5, __OnOPTO3Contact);


  ros::Publisher in_virtuose_force = n.advertise<virtuose::in_virtuose_force>("in_virtuose_force", 1);
  ros::ServiceClient virtuose_impedance = n.serviceClient<virtuose::virtuose_impedance>("virtuose_impedance"); ////cosa e' questo?
  ros::ServiceClient virtuose_reset = n.serviceClient<virtuose::virtuose_reset>("virtuose_reset");

// Reset virtuose_node
  virtuose::virtuose_reset res;
  // virtuose_reset.call(res);
  
// Create multithread spinner
  ros::AsyncSpinner spinner(2);
  spinner.start();

  uint64_t start = ros::Time::now().toNSec();

// Request impedance mode
  printf("Sending impedance request\n");
  virtuose::virtuose_impedance imp;
  imp.request.ip_address = "localhost#53210";
  imp.request.indexing_mode = 0;
  imp.request.speed_factor = 1.0;
  imp.request.force_factor = 1.0;
  imp.request.power_enable = 1;
  imp.request.max_force = 10.0;
  imp.request.max_torque = 1.0;
  imp.request.base_frame.translation.x = 0.0;
  imp.request.base_frame.translation.y = 0.0;
  imp.request.base_frame.translation.z = 0.0;
  imp.request.base_frame.rotation.x = 0.0;
  imp.request.base_frame.rotation.y = 0.0;
  imp.request.base_frame.rotation.z = 0.0;
  imp.request.base_frame.rotation.w = 1.0;
  if (!virtuose_impedance.call(imp))
  {
    printf("Call failed\n");
    return 0;
  }
// Store client ID given by virtuose_node
  uint32_t client_id = 0;
  client_id = imp.response.client_id;
  printf("Our client ID is %d\n", client_id);



/*

IMPEDANCE CONTROLLER

*/


// Perform impedance loop at 500 Hz
  ros::Rate r(500);
  int ctr = 0;
  while(ros::ok())
  {
// Create null force
    virtuose::in_virtuose_force force;
    force.header.stamp = ros::Time::now();
    force.client_id = client_id;
    force.virtuose_force.force.x = 0.0;
    force.virtuose_force.force.y = 0.0;
    force.virtuose_force.force.z = 0.0;
    force.virtuose_force.torque.x = 0.0;
    force.virtuose_force.torque.y = 0.0;
    force.virtuose_force.torque.z = 0.0;

    
//DELTA TRANSLATION NO ROTATION
    for(int j=0;j<3;j++)
    {
      delta_pos_H[j]=cur_pose_H[j] - old_pose[j];
      delta_vel_H[j]=delta_pos_H[j]/dt;
      old_pose[j]=cur_pose_H[j];
    }
    
//CURRENT/OLD DELTA POSITION OF UR5
    for(int k=0;k<3;k++)
    {
      //delta_pos_ur5[k]=cur_position_ur5[k] - old_position_ur5[k];
      if(cur_position_ur5[k]!=old_position_ur5[k] && delta_pos_H[k] != 0 )
        delta_pos_ur5[k]=cur_position_ur5[k] - old_position_ur5[k];
      

      //delta_vel_ur5[k]=delta_pos_ur5[k]/dt;
      
      if (cur_position_ur5[k] == old_position_ur5[k] && delta_pos_H[k] != 0)
        delta_pos_ur5[k]=delta_pos_ur5_old[k];
      else
        delta_pos_ur5_old[k]=delta_pos_ur5[k];
      //virutal_elong__ur5[k]=initial_position_ur5[k] - cur_position_ur5[k];
      old_position_ur5[k]=cur_position_ur5[k];

    }

    myfile << "cur_pose_H"<< "," <<cur_pose_H[0]<< "," << cur_pose_H[1]<< "," <<cur_pose_H[2]<< ","  ;
    myfile << "old_pose_H"<< "," <<old_pose[0]<< "," << old_pose[1]<< "," <<old_pose[2]<< ","  ;
    myfile <<  "cur_position_ur5 " << "," <<cur_position_ur5[0] << "," <<cur_position_ur5[1] << "," <<cur_position_ur5[2] << "," ;
    myfile <<  "old_position_ur5 " << "," <<old_position_ur5[0] << "," <<old_position_ur5[1] << "," <<old_position_ur5[2] << "," ;

    myfile << "delta_pos H"<< "," <<delta_pos_H[0]<< "," << delta_pos_H[1]<< "," <<delta_pos_H[2]<< ",";
    myfile << "delta_pos UR5" << "," <<delta_pos_ur5[0] << "," <<delta_pos_ur5[1] << "," <<delta_pos_ur5[2] << "\n";


//ERROR BETWEEN VIRTUOSE AND UR5 time-step by step
error_x=delta_pos_H[0] - delta_pos_ur5[0];//*100; //-x to -x
error_y=delta_pos_H[1] + delta_pos_ur5[1];//*100; // y to z
error_z=delta_pos_H[2] - delta_pos_ur5[2];//*100; // z to y
  //std::cout << "error_x: " << error_x << " error_y: " << error_y << " error_z: " << error_z << "\n";
  //printf("error x %f,error y %f,error z %f \n", error_x, error_y, error_z);
//std::cout << "delta_pos_H[0]: " << delta_pos_H[0] << " delta_pos_H[1]: " << delta_pos_H[1] << " delta_pos_H[2]: " << delta_pos_H[2] << "\n";
//std::cout << "delta_pos_ur5:[0] " << delta_pos_ur5[0] << " delta_pos_ur5[1]: " << delta_pos_ur5[1] << " delta_pos_ur5[2]: " << delta_pos_ur5[2] << "\n";

//VIRTUAL ELONG FIXED FRAME UR5
virutal_elong__ur5[0]=-(initial_position_ur5[0]-cur_position_ur5[0]);
virutal_elong__ur5[1]=-(initial_position_ur5[1]-cur_position_ur5[1]);
virutal_elong__ur5[2]=-(initial_position_ur5[2]-cur_position_ur5[2]);
  //printf("virutal_elong__ur5[1] %f\n",virutal_elong__ur5[1]);
  //printf("sign ur5 %d \n",sgn(virutal_elong__ur5[2]));

//VIRTUAL ELONG FIXED FRAME VIRTUOSE
virutal_elong__H[0]=-(initial_pose_H[0]-cur_pose_H[0]);
virutal_elong__H[1]=-(initial_pose_H[1]-cur_pose_H[1]);
virutal_elong__H[2]=-(initial_pose_H[2]-cur_pose_H[2]);
  //printf("virutal_elong__H[1] %f\n",virutal_elong__H[1]);


    enable_virtual_elong_ur5=false;//false=0 true=1 //false-> Haption Spring;    true-> Haption-UR5 spring

fixed_spring_x=virutal_elong__H[0]-(virutal_elong__ur5[0]*enable_virtual_elong_ur5);
fixed_spring_y=virutal_elong__H[1]+(virutal_elong__ur5[1]*enable_virtual_elong_ur5); //+ cause H and ur5 opposite sign
fixed_spring_z=virutal_elong__H[2]-(virutal_elong__ur5[2]*enable_virtual_elong_ur5);
  //printf("virutal_elong__H[0]-virutal_elong__ur5[0] = spring_x: %f - %f = %f \n",virutal_elong__H[0],virutal_elong__ur5[0],spring_x);
  //printf("virutal_elong__H[1]-virutal_elong__ur5[1] = spring_y: %f + %f = %f \n",virutal_elong__H[1],virutal_elong__ur5[1],spring_y);
  //printf("virutal_elong__H[2]-virutal_elong__ur5[2] = spring_z: %f - %f = %f \n \n",virutal_elong__H[2],virutal_elong__ur5[2],spring_z);
    
    force_limit=5;
    KK_fix=100;// 100 STRONG FORCE, 10 LIGHT FORCE 
    KK_curr=0;//150;
    enable_fixed_spring_effect=true;
    enable_curr_spring_effect=false;

fixed_spring_x_force=KK_fix*(fixed_spring_x)*enable_fixed_spring_effect;
fixed_spring_y_force=KK_fix*(fixed_spring_y)*enable_fixed_spring_effect;
fixed_spring_z_force=KK_fix*(fixed_spring_z)*enable_fixed_spring_effect;

curr_spring_x_force=KK_curr*(error_x)*enable_curr_spring_effect;//*10;
curr_spring_y_force=KK_curr*(error_y)*enable_curr_spring_effect;//*10;
curr_spring_z_force=KK_curr*(error_z)*enable_curr_spring_effect;//*10;

spring_force[0]=-(fixed_spring_x_force + curr_spring_x_force);//-(error_sensitivity*sgn(spring_x)));//
spring_force[1]=-(fixed_spring_y_force + curr_spring_y_force);//-(error_sensitivity*sgn(spring_y)));//  
spring_force[2]=-(fixed_spring_z_force + curr_spring_z_force);//-(error_sensitivity*sgn(spring_z)));//

for (int t=0;t<=3;t++)
{
 if (abs(spring_force[t])>force_limit)
    {
      std::cout << std::setprecision(5);

      //std::cout << "fixed_spring_x_force: -- x : -- " << fixed_spring_x_force << " -- fixed_spring_y_force : -- " << fixed_spring_y_force << "-- fixed_spring_z_force : -- " << fixed_spring_z_force << "\n";
     // std::cout << "curr_spring_x_force: -- x : -- " << curr_spring_x_force << " -- curr_spring_y_force : -- " << curr_spring_y_force << "-- curr_spring_z_force : -- " << curr_spring_z_force << "\n";
    }
}


if(enable_fixed_spring_effect==true && enable_curr_spring_effect==false )
    std::cout << std::setprecision(2);
    
    //std::cout << "fixed_spring_x: -- x : -- " << fixed_spring_x << " -- fixed_spring_y : -- " << fixed_spring_y << "-- fixed_spring_z : -- " << fixed_spring_z << "\n";
else if(enable_curr_spring_effect==true && enable_fixed_spring_effect==false)
  {
    std::cout << std::setprecision(5);
    //std::cout << "error_x: -- " << error_x << " error_y: -- " << error_y << " error_z: -- " << error_z << "\n";
    std::cout << std::setprecision(2);
  }
else if(enable_curr_spring_effect==false && enable_fixed_spring_effect==false)
  printf("WARNING : NO SPRING FORCES ARE APPLIED");
else if(enable_curr_spring_effect==true && enable_fixed_spring_effect==true)
  printf("WARNING : BOTH SPRING FORCES ARE APPLIED");
  
//std::cout << "Spring force: -- x : -- " << spring_force[0] << " -- y : -- " << spring_force[1] << "-- z : -- " << spring_force[2]<< "\n";




/*

BELOW FORCE SETTINGS AND SAFETY CHECKS

*/

//GENERATE X FORCE
error_sensitivity=0;
      if (abs(error_x) > error_sensitivity)
      {
        //spring_force[0]=virutal_elong__H[0]-virutal_elong__ur5[0];
        //damping_force[0] =-(error_x-(error_sensitivity*sgn(error_x)))*damping_coeff;
        force.virtuose_force.force.x = spring_force[0];
        if (force.virtuose_force.force.x > force_limit) //10 is a safety treshold
        {
          force.virtuose_force.force.x = force_limit;
          spring_force[0]=force_limit;
          //printf("WARNING: FORCE TRESHOLD X REACHED %f, SETTING SAFETY VALUE %f \n", spring_force[0], force_limit);

        }
        else if (force.virtuose_force.force.x < -force_limit)
        {
          force.virtuose_force.force.x = -force_limit;
          spring_force[0]=-force_limit;
          //printf("WARNING: FORCE TRESHOLD X REACHED %f, SETTING SAFETY VALUE -%f \n", spring_force[0], force_limit);
        }     
        // else
        // printf("");
        //printf("X FORCE= %f CCC \n", spring_force[0]);
      }
     else
     {
      //printf("X FORCE LIMIT NOT REACHED \n ");
      //printf("DAMPING FORCE X NULL\n");
      spring_force[0]=0.0;
      force.virtuose_force.force.x = 0;   
     }


//GENERATE Y FORCE
      if (abs(error_y) > error_sensitivity)
      {
        //spring_force[1]=virutal_elong__H[1]-virutal_elong__ur5[1];
        //damping_force[1]=-(error_y-(error_sensitivity*sgn(error_y)))*damping_coeff;

        force.virtuose_force.force.y = spring_force[1];/////////////////////////////////
        if (force.virtuose_force.force.y > force_limit) //10 is a safety treshold
        {
          force.virtuose_force.force.y = force_limit;
          spring_force[1]=force_limit;
          //printf("WARNING: FORCE TRESHOLD Y REACHED %f, SETTING SAFETY VALUE %f \n", spring_force[1],force_limit);

        }
        else if (force.virtuose_force.force.y < -force_limit)
        {
          force.virtuose_force.force.y = -force_limit;
          spring_force[1]=-force_limit;
          //printf("WARNING: FORCE TRESHOLD Y REACHED %f, SETTING SAFETY VALUE -%f \n", spring_force[1], force_limit);

        }  
        // else
        // printf("");
        //printf("Y FORCE=%f BBB \n",spring_force[1]);
      }
      
     else
     {
      //printf("Y FORCE LIMIT NOT REACHED \n ");
      //printf("DAMPING FORCE Y NULL \n");
      spring_force[1]=0.0;

      force.virtuose_force.force.y = 0;   
     }


//GENERATE Z FORCE
      if (abs(error_z) > error_sensitivity)
      {
        //spring_force[2]=virutal_elong__H[2]-virutal_elong__ur5[2];
        //damping_force[2]=-(error_z-(error_sensitivity*sgn(error_z)))*damping_coeff;
      
        force.virtuose_force.force.z = spring_force[2];///////////////////////////////////////////////
        if (force.virtuose_force.force.z > force_limit) //10 is a safety treshold
        {
          force.virtuose_force.force.z = force_limit;
          spring_force[2]=force_limit;
          //printf("WARNING: FORCE TRESHOLD Z REACHED %f, SETTING SAFETY VALUE %f \n", spring_force[2], force_limit);
        }  
        else if (force.virtuose_force.force.z < -force_limit)
        {
          force.virtuose_force.force.z = -force_limit;
          spring_force[2]=-force_limit;
          //printf("WARNING: FORCE TRESHOLD Z REACHED %f, SETTING SAFETY VALUE -%f \n", spring_force[2], force_limit);
        }  
        // else
        // printf("");
        //printf("Z FORCE=%f AAA \n",spring_force[2]);
      }
     else
     {

      //printf("Z FORCE LIMIT NOT REACHED \n");
      //printf("DAMPING FORCE Z NULL \n");
      spring_force[2]=0.0;
      force.virtuose_force.force.z = 0;   
     }
       
//PUBLISH FORCE
    test_x=force.virtuose_force.force.x;
    test_y=force.virtuose_force.force.y;
    test_z=force.virtuose_force.force.z;


    if (abs(test_x)>force_limit || abs(test_y)>force_limit || abs(test_z)>force_limit || sensor_contact==true)
      {
        printf("DANGEROUS FORCE FEEDBACK  TURN OFF THE GREEN BUTTON ");
        force.virtuose_force.force.x=0.0;
        force.virtuose_force.force.y = 0.0;
        force.virtuose_force.force.z=0.0;
      }

    
    //std::cout << "FORCE PUB -- x : -- " << force.virtuose_force.force.x << " -- y : -- " << force.virtuose_force.force.y << "-- z : -- " << force.virtuose_force.force.z<< "\n \n";
    in_virtuose_force.publish(force);
    
    ctr ++;
    // printf status every second
    // if (ctr % 500 == 0)
    // {
    //   printf("");
    //   //printf("Status: %ld %d %d %f %f %f\n", status_date - start, status_state, status_button,
    //    //      cur_pose_H[0], cur_pose_H[1], cur_pose_H[2]);
    // }
    r.sleep();
  }

// Reset virtuose_node
  virtuose_reset.call(res);

  return 0;
}




//il problema e' che questi due valori sono in due spazi diversi e quindi forse non comparabili, bisognerebbe normalizzarli (sigmoide?)

    //quando e' necessario usare la velocita?
    //cosa accade se i due dt sono diversi?
    //chi pubblica questi valori?

    //struttura dati letti da virtuose: cur_pose_H viene salvata gia in questo file, bisogna salvare anche quella precedente
    /*topic =  #questo cambia anche senza applicare forza sul manettino
    virtuose_physical_pose: 
    translation: 
    x: 0.603905320168
    y: 0.412554621696
    z: -0.041582185775
    rotation: 
    x: 0.29095056653
    y: 0.612622022629
    z: -0.18339882791
    w: 0.711622714996
    */
   /*topic =#questo cambia solo se si impugna con decisione il manettino
    virtuose_pose: 
    translation: 
    x: 0.750722944736
    y: 0.209120973945
    z: 0.0165340080857
    rotation: 
    x: 0.386401057243
    y: 0.554185032845
    z: -0.311827450991
    w: 0.668084442616
    */