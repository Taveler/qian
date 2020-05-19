#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/keyboard.h>
#include <webots/gps.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/camera_recognition_object.h>
#include <webots/display.h>
#include <webots/compass.h>
#include <webots/distance_sensor.h>


#define GRIPPER_MOTOR_MAX_SPEED 1
#define TIME_STEP 16
#define LIGHT_GRAY 0x505050
#define RED 0xBB2222
#define GREEN 0x22BB11
#define BLUE 0x2222BB
#define YELLOW 0xFFFF00
#define MAX_SPEED 3

#define NORTH 1.5708
#define WEST 3.1416
#define SOUTH -1.5708
#define EAST 0.0

#define floor0 0
#define floor1 -0.2
#define floor2 -0.4

#define UngripPosLeftMid 2
#define UngripPosRightMid 58
#define UngripPosLeftBig 5
#define UngripPosRightBig 55
#define UngripPosLeftSmall 1
#define UngripPosRightSmall 61



static WbDeviceTag wheel_motors[3];
static WbDeviceTag gripper_motors[3];
static WbDeviceTag camera;
static WbDeviceTag cameraL;
static WbDeviceTag cameraR;
static WbDeviceTag gps;
static WbDeviceTag ground_display;
static WbDeviceTag compass;
static WbDeviceTag distance_sensor;
static WbDeviceTag so0;
static WbDeviceTag so1;
static WbDeviceTag so2;
static WbDeviceTag so3;
static WbDeviceTag so4;
static WbDeviceTag so5;
static WbDeviceTag so6;
static WbDeviceTag so7;
static WbDeviceTag so8;
static WbDeviceTag so15;
void DisplayMap()
{
// get the properties of the Display
  int width = wb_display_get_width(ground_display);
  int height = wb_display_get_height(ground_display);
  // paint the display's background
  wb_display_set_color(ground_display, LIGHT_GRAY);
  wb_display_fill_rectangle(ground_display, 0, 0, width, height);
  wb_display_set_color(ground_display, RED);
  wb_display_draw_line(ground_display, 0, height / 2, width - 1, height / 2);
  wb_display_draw_text(ground_display, "x", width - 10, height / 2 - 10);
  wb_display_set_color(ground_display, BLUE);
  wb_display_draw_line(ground_display, width / 2, 0, width / 2, height - 1);
  wb_display_draw_text(ground_display, "z", width / 2 - 10, height - 10);
  wb_display_set_color(ground_display, GREEN);
  wb_display_draw_text(ground_display, "起点", 35, 35);
  wb_display_set_color(ground_display, RED);
  wb_display_fill_rectangle(ground_display, 70, 70, 60, 60);
  wb_display_set_color(ground_display, BLUE);
  wb_display_fill_rectangle(ground_display, 0, 50, 15, 100);
  wb_display_fill_rectangle(ground_display, 185, 50, 15, 100);
  wb_display_fill_rectangle(ground_display, 50, 0, 100, 15);
  wb_display_fill_rectangle(ground_display, 50, 185, 100, 15);
  
  const double *gps_values = wb_gps_get_values(gps); 
  wb_display_set_color(ground_display, YELLOW);
  wb_display_fill_rectangle(ground_display, gps_values[0]*50+95, gps_values[2]*50+95, 10, 10);
  
  //const double *compass_values = wb_compass_get_values(compass); 
  //printf("Using compass: %.3f %.3f\n", compass_values[0], compass_values[2]); 
 }

static void initialize() {
  /* necessary to initialize Webots */
  wb_robot_init();

  gripper_motors[0] = wb_robot_get_device("lift motor");
  gripper_motors[1] = wb_robot_get_device("left finger motor");
  gripper_motors[2] = wb_robot_get_device("right finger motor");
  wheel_motors[0] = wb_robot_get_device("left wheel");
  wheel_motors[1] = wb_robot_get_device("right wheel");
  // Specify velocity control mode
  wb_motor_set_position(wheel_motors[0], INFINITY);
  wb_motor_set_position(wheel_motors[1], INFINITY);
  wb_motor_set_velocity(wheel_motors[0], 0.0);
  wb_motor_set_velocity(wheel_motors[1], 0.0);
  
  gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);
  
  wb_keyboard_enable(TIME_STEP);
  
  camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);
  wb_camera_recognition_enable(camera, TIME_STEP);
  cameraL = wb_robot_get_device("cameraL");
  wb_camera_enable(cameraL, TIME_STEP);
  wb_camera_recognition_enable(cameraL, TIME_STEP);
  cameraR = wb_robot_get_device("cameraR");
  wb_camera_enable(cameraR, TIME_STEP);
  wb_camera_recognition_enable(cameraR, TIME_STEP);
  
  // to draw the overlay information
  ground_display = wb_robot_get_device("display");
  
  compass = wb_robot_get_device("compass");
  wb_compass_enable(compass, TIME_STEP);
  
  distance_sensor = wb_robot_get_device("distance sensor");
  wb_distance_sensor_enable(distance_sensor, TIME_STEP);
  
  so1 = wb_robot_get_device("so1");
  wb_distance_sensor_enable(so1, TIME_STEP);
  so2 = wb_robot_get_device("so2");
  wb_distance_sensor_enable(so2, TIME_STEP);
  so3 = wb_robot_get_device("so3");
  wb_distance_sensor_enable(so3, TIME_STEP);
  so4 = wb_robot_get_device("so4");
  wb_distance_sensor_enable(so4, TIME_STEP);
  so5 = wb_robot_get_device("so5");
  wb_distance_sensor_enable(so5, TIME_STEP);
  so6 = wb_robot_get_device("so6");
  wb_distance_sensor_enable(so6, TIME_STEP);
  so0 = wb_robot_get_device("so0");
  wb_distance_sensor_enable(so0, TIME_STEP);
  so7 = wb_robot_get_device("so7");
  wb_distance_sensor_enable(so7, TIME_STEP);
  so8 = wb_robot_get_device("so8");
  wb_distance_sensor_enable(so8, TIME_STEP);
  so15 = wb_robot_get_device("so15");
  wb_distance_sensor_enable(so15, TIME_STEP);
}

void step(double seconds) {
  const double ms = seconds * 1000.0;
  int elapsed_time = 0;
  while (elapsed_time < ms) {
    wb_robot_step(TIME_STEP);
    elapsed_time += TIME_STEP;
  }
}

void lift(double position) {
  wb_motor_set_velocity(gripper_motors[0], GRIPPER_MOTOR_MAX_SPEED);
  wb_motor_set_position(gripper_motors[0], position);
}

void moveFingers(double position) {
  wb_motor_set_velocity(gripper_motors[1], GRIPPER_MOTOR_MAX_SPEED);
  wb_motor_set_velocity(gripper_motors[2], GRIPPER_MOTOR_MAX_SPEED);
  wb_motor_set_position(gripper_motors[1], position);
  wb_motor_set_position(gripper_motors[2], position);
}

void moveForwards(double speed) {
  wb_motor_set_velocity(wheel_motors[0], speed);
  wb_motor_set_velocity(wheel_motors[1], speed);
}

void turn(double speed) {
  wb_motor_set_velocity(wheel_motors[0], speed);
  wb_motor_set_velocity(wheel_motors[1], -speed);
}

void stop(double seconds) {
  wb_motor_set_velocity(wheel_motors[0], 0.0);
  wb_motor_set_velocity(wheel_motors[1], 0.0);
  step(seconds);
}

bool oam_on = 1;
void oam()
{
   int ds_value0 = wb_distance_sensor_get_value(so0);
   int ds_value1 = wb_distance_sensor_get_value(so1);
   int ds_value2 = wb_distance_sensor_get_value(so2);
   int ds_value3 = wb_distance_sensor_get_value(so3);
   int ds_value4 = wb_distance_sensor_get_value(so4);
   int ds_value5 = wb_distance_sensor_get_value(so5);
   int ds_value6 = wb_distance_sensor_get_value(so6);
   int ds_value7 = wb_distance_sensor_get_value(so7);
   int ds_value8 = wb_distance_sensor_get_value(so8);
   int ds_value15 = wb_distance_sensor_get_value(so15);
   while((ds_value0>50||ds_value1>50||ds_value2>50||ds_value3>50||ds_value15>50||
   ds_value4>50||ds_value5>50||ds_value6>50||ds_value7>50||ds_value8>50)
   && wb_robot_step(TIME_STEP) != 1)
   {
     DisplayMap();
     double DirectionCtrl = (ds_value0+ds_value1+ds_value2+ds_value3+ds_value15
     -ds_value4-ds_value5-ds_value6-ds_value7-ds_value8)/1500.0*MAX_SPEED;
     wb_motor_set_velocity(wheel_motors[0], 0.5*MAX_SPEED + DirectionCtrl);
     wb_motor_set_velocity(wheel_motors[1], 0.5*MAX_SPEED - DirectionCtrl);
     ds_value0 = wb_distance_sensor_get_value(so0);
     ds_value1 = wb_distance_sensor_get_value(so1);
     ds_value2 = wb_distance_sensor_get_value(so2);
     ds_value3 = wb_distance_sensor_get_value(so3);
     ds_value4 = wb_distance_sensor_get_value(so4);
     ds_value5 = wb_distance_sensor_get_value(so5);
     ds_value6 = wb_distance_sensor_get_value(so6);
     ds_value7 = wb_distance_sensor_get_value(so7);
     ds_value8 = wb_distance_sensor_get_value(so8);
     ds_value15 = wb_distance_sensor_get_value(so15);
   }
}

double Recognize(char* target)
{
    int number_of_objects = wb_camera_recognition_get_number_of_objects(camera);
    //printf("Recognized %d objects.\n", number_of_objects);
   
    const WbCameraRecognitionObject *objects = wb_camera_recognition_get_objects(camera);
    for (int i = 0; i < number_of_objects; ++i)
    {
      //printf("Model of object %d: %s\n", i, objects[i].model);
      /*
      printf("Id of object %d: %d\n", i, objects[i].id);
      printf("Relative position of object %d: %lf %lf %lf\n", i, objects[i].position[0], objects[i].position[1],
             objects[i].position[2]);
      printf("Relative orientation of object %d: %lf %lf %lf %lf\n", i, objects[i].orientation[0], objects[i].orientation[1],
             objects[i].orientation[2], objects[i].orientation[3]);
      printf("Size of object %d: %lf %lf\n", i, objects[i].size[0], objects[i].size[1]);
      
      printf("Position of the object %d on the camera image: %d %d\n", i, objects[i].position_on_image[0],
             objects[i].position_on_image[1]);
      printf("Size of the object %d on the camera image: %d %d\n", i, objects[i].size_on_image[0], objects[i].size_on_image[1]);
      for (j = 0; j < objects[i].number_of_colors; ++j)
        printf("- Color %d/%d: %lf %lf %lf\n", j + 1, objects[i].number_of_colors, objects[i].colors[3 * j],
               objects[i].colors[3 * j + 1], objects[i].colors[3 * j + 2]);
               */
      if(strcmp(target,objects[i].model) == 0 && 
      (objects[i].position_on_image[0]<150 && 
      objects[i].position_on_image[0]>106 && 
      objects[i].size_on_image[0]>32))// || objects[i].size_on_image[0]>90)
      {
        return objects[i].position_on_image[0]/100.0;
      }
    }
    return -1.0;
  
}

void SetDirection(double TargetAngel)
{
  DisplayMap();
  const double *compass_values = wb_compass_get_values(compass); 
  double SelfAngle = atan2(compass_values[0], -compass_values[2]);
  while((fabs(TargetAngel-SelfAngle)>0.01 || (fabs(TargetAngel)>3.14 && fabs(SelfAngle)>3.14))  && wb_robot_step(TIME_STEP) != 1)
  {
    double DirectionCtrl = (TargetAngel-SelfAngle)*0.5;
    if(TargetAngel>SelfAngle)
      DirectionCtrl = MAX_SPEED;
    else
      DirectionCtrl = -MAX_SPEED;
    if(TargetAngel>1.571 && SelfAngle<-1.571)
    {
      DirectionCtrl = -MAX_SPEED;
    }
    if(TargetAngel<-1.571 && SelfAngle>1.571)
    {
      DirectionCtrl = MAX_SPEED;
    }
   
    wb_motor_set_velocity(wheel_motors[0], - DirectionCtrl);
    wb_motor_set_velocity(wheel_motors[1], DirectionCtrl);
    
    compass_values = wb_compass_get_values(compass);
    SelfAngle = atan2(compass_values[0], -compass_values[2]);
    DisplayMap();
  }
  stop(0);
}
void GoToPos(double* TargetPos)
{
  DisplayMap();
  const double *compass_values = wb_compass_get_values(compass); 
  const double *gps_values = wb_gps_get_values(gps);
  
  double SelfAngle = atan2(compass_values[0], -compass_values[2]);
  double TargetAngel = atan2(gps_values[2]-TargetPos[1],TargetPos[0]-gps_values[0]);
  
  SetDirection(TargetAngel);
  
  static bool TurnFlag = 1;
  
  while((fabs(gps_values[2]-TargetPos[1]) > 0.08 || fabs(TargetPos[0]-gps_values[0]) > 0.08)  && wb_robot_step(TIME_STEP) != 1)
  {
  if(oam_on)
  {
    int ds_value0 = wb_distance_sensor_get_value(so0);
   int ds_value1 = wb_distance_sensor_get_value(so1);
   int ds_value2 = wb_distance_sensor_get_value(so2);
   int ds_value3 = wb_distance_sensor_get_value(so3);
   int ds_value4 = wb_distance_sensor_get_value(so4);
   int ds_value5 = wb_distance_sensor_get_value(so5);
   int ds_value6 = wb_distance_sensor_get_value(so6);
   int ds_value7 = wb_distance_sensor_get_value(so7);
     if(ds_value0>200||ds_value1>200||ds_value2>200||ds_value3>200||
   ds_value4>200||ds_value5>200||ds_value6>200||ds_value7>200)
       {
         oam();
         compass_values = wb_compass_get_values(compass);
        gps_values = wb_gps_get_values(gps);
        
        SelfAngle = atan2(compass_values[0], -compass_values[2]);
        TargetAngel = atan2(gps_values[2]-TargetPos[1],TargetPos[0]-gps_values[0]);
        SetDirection(TargetAngel);
       }
   }
     
      if(TurnFlag)
    {
      if(fabs(gps_values[0])<1 && fabs(gps_values[2])<0.9 )//fabs(gps_values[0])>0.98 && 
      {
        double NextPos[2]={gps_values[0]*1.05,TargetPos[1]};
        TurnFlag = 0;
        GoToPos(NextPos);
        compass_values = wb_compass_get_values(compass); 
        gps_values = wb_gps_get_values(gps);
        SelfAngle = atan2(compass_values[0], -compass_values[2]);
        TargetAngel = atan2(gps_values[2]-TargetPos[1],TargetPos[0]-gps_values[0]);
        SetDirection(TargetAngel);
      }
      else
        if(fabs(gps_values[0])<0.9 &&fabs(gps_values[2])<1)// fabs(gps_values[2])>0.98 && 
        {
          double NextPos[2]={TargetPos[0],gps_values[2]*1.05};
          TurnFlag = 0;
          GoToPos(NextPos);
          compass_values = wb_compass_get_values(compass); 
          gps_values = wb_gps_get_values(gps);
          SelfAngle = atan2(compass_values[0], -compass_values[2]);
          TargetAngel = atan2(gps_values[2]-TargetPos[1],TargetPos[0]-gps_values[0]);
          SetDirection(TargetAngel);
        }
    }
    double DirectionCtrl = (TargetAngel-SelfAngle)*0.3;
    if(TargetAngel>3 && SelfAngle<-3)
    {
      DirectionCtrl = -MAX_SPEED*0.3;
    }
    if(TargetAngel<-3 && SelfAngle>3)
    {
      DirectionCtrl = MAX_SPEED*0.3;
    }
    
    wb_motor_set_velocity(wheel_motors[0], MAX_SPEED - DirectionCtrl);
    wb_motor_set_velocity(wheel_motors[1], MAX_SPEED + DirectionCtrl);
    
    //printf("Using the GPS device: %.3f %.3f\n", gps_values[0], gps_values[2]); 
    compass_values = wb_compass_get_values(compass);
    gps_values = wb_gps_get_values(gps);
    
    SelfAngle = atan2(compass_values[0], -compass_values[2]);
    TargetAngel = atan2(gps_values[2]-TargetPos[1],TargetPos[0]-gps_values[0]); 
    //printf("TargetAngel: %.3f SelfAngle: %.3f \n", TargetAngel,SelfAngle);
    
    DisplayMap();
  }
  TurnFlag = 1;
  stop(0);
}

int AutoGrip(char* target)
{
  DisplayMap();
  const double *gps_values = wb_gps_get_values(gps);
  double SelfPos = atan2(gps_values[2], gps_values[0]);
  if(fabs(SelfPos)<0.785)
    SetDirection(WEST);
    else
      if(fabs(SelfPos)>2.356)
        SetDirection(EAST);
      else
        if(SelfPos>0)
          SetDirection(NORTH);
        else
          SetDirection(SOUTH);
          
         
  //bool noTarget = 0;
  moveFingers(0.08);
  lift(0.05);
  int ds_value = wb_distance_sensor_get_value(distance_sensor);
  while(ds_value>130 && wb_robot_step(TIME_STEP) != 1)
  {
    
    double target_Pos = Recognize(target);
    if(target_Pos<0)
    {
      //noTarget = 1;
      target_Pos = 1.28;
     }
    target_Pos -= 1.28;
    wb_motor_set_velocity(wheel_motors[0], 0.6*MAX_SPEED+target_Pos);
    wb_motor_set_velocity(wheel_motors[1], 0.6*MAX_SPEED-target_Pos);
    
    //wb_motor_set_velocity(wheel_motors[0], 0.6*MAX_SPEED);
    //wb_motor_set_velocity(wheel_motors[1], 0.6*MAX_SPEED);
    ds_value = wb_distance_sensor_get_value(distance_sensor);
    //printf("target_Pos: %3f\n",target_Pos);
    DisplayMap();
  }
  
  //if(noTarget)
    //return -1;
  //else
  {  
    moveFingers(0.01);
    stop(0.5);
    lift(0);
    stop(0.3);
    const double *gps_values = wb_gps_get_values(gps);
    while(gps_values[0]>-1.18 && gps_values[2]>-1.18 &&gps_values[0]<1.18 &&gps_values[2]<1.18 && wb_robot_step(TIME_STEP) != 1)
    {
      moveForwards(-MAX_SPEED*0.6);
      gps_values = wb_gps_get_values(gps);
      //printf("Using the GPS device: %.3f %.3f\n", gps_values[0], gps_values[2]);
      DisplayMap();
    }
    stop(0);
    return 1;
  }
}

void Ungrip(double floor)
{
  DisplayMap();
  const double *gps_values = wb_gps_get_values(gps);
  double SelfPos = atan2(gps_values[2], gps_values[0]);
  if(fabs(SelfPos)<0.785)
    SetDirection(EAST);
    else
      if(fabs(SelfPos)>2.356)
        SetDirection(WEST);
      else
        if(SelfPos>0)
          SetDirection(SOUTH);
        else
          SetDirection(NORTH);
  lift(floor);
  
  while(gps_values[0]>-1.47 && gps_values[2]>-1.47 &&gps_values[0]<1.47 &&gps_values[2]<1.47 && wb_robot_step(TIME_STEP) != 1)
  {
    moveForwards(MAX_SPEED*0.6);
    gps_values = wb_gps_get_values(gps);
    //printf("Using the GPS device: %.3f %.3f\n", gps_values[0], gps_values[2]);
    DisplayMap();
  }
  stop(0);
  moveFingers(0.08);
  stop(0.3);
  while((gps_values[0]<-1.22 || gps_values[2]<-1.22 || gps_values[0]>1.22 || gps_values[2]>1.22) && wb_robot_step(TIME_STEP) != 1)
  {
    moveForwards(-MAX_SPEED*0.6);
    gps_values = wb_gps_get_values(gps);
    //printf("Using the GPS device: %.3f %.3f\n", gps_values[0], gps_values[2]);
    DisplayMap();
  }
  stop(0);
  lift(0.05);
}

int UngripPosLeft = 4;
int UngripPosRight = 60;

void GoToPosUngrip(double* TargetPos,char *target)
{
  DisplayMap();
  const double *compass_values = wb_compass_get_values(compass); 
  const double *gps_values = wb_gps_get_values(gps);
  
  double SelfAngle = atan2(compass_values[0], -compass_values[2]);
  double TargetAngel = atan2(gps_values[2]-TargetPos[1],TargetPos[0]-gps_values[0]);
  
  SetDirection(TargetAngel);
  
  bool UngripFlag = 0;
  
  while((fabs(gps_values[2]-TargetPos[1]) > 0.08 || fabs(TargetPos[0]-gps_values[0]) > 0.08)  && wb_robot_step(TIME_STEP) != 1)
  {
    if(oam_on)
  {
    int ds_value0 = wb_distance_sensor_get_value(so0);
   int ds_value1 = wb_distance_sensor_get_value(so1);
   int ds_value2 = wb_distance_sensor_get_value(so2);
   int ds_value3 = wb_distance_sensor_get_value(so3);
   int ds_value4 = wb_distance_sensor_get_value(so4);
   int ds_value5 = wb_distance_sensor_get_value(so5);
   int ds_value6 = wb_distance_sensor_get_value(so6);
   int ds_value7 = wb_distance_sensor_get_value(so7);
     if(ds_value0>200||ds_value1>200||ds_value2>200||ds_value3>200||
   ds_value4>200||ds_value5>200||ds_value6>200||ds_value7>200)
       {
         oam();
         compass_values = wb_compass_get_values(compass);
        gps_values = wb_gps_get_values(gps);
        
        SelfAngle = atan2(compass_values[0], -compass_values[2]);
        TargetAngel = atan2(gps_values[2]-TargetPos[1],TargetPos[0]-gps_values[0]);
        SetDirection(TargetAngel);
       }
   }
   
    double DirectionCtrl = (TargetAngel-SelfAngle)*0.3;
    if(TargetAngel>3 && SelfAngle<-3)
    {
      DirectionCtrl = -MAX_SPEED*0.3;
    }
    if(TargetAngel<-3 && SelfAngle>3)
    {
      DirectionCtrl = MAX_SPEED*0.3;
    }
    
    wb_motor_set_velocity(wheel_motors[0], MAX_SPEED - DirectionCtrl);
    wb_motor_set_velocity(wheel_motors[1], MAX_SPEED + DirectionCtrl);
    
    //printf("Using the GPS device: %.3f %.3f\n", gps_values[0], gps_values[2]); 
    compass_values = wb_compass_get_values(compass);
    gps_values = wb_gps_get_values(gps);
    
    SelfAngle = atan2(compass_values[0], -compass_values[2]);
    TargetAngel = atan2(gps_values[2]-TargetPos[1],TargetPos[0]-gps_values[0]); 
    //printf("TargetAngel: %.3f SelfAngle: %.3f \n", TargetAngel,SelfAngle);
    DisplayMap();
    
    int number_of_objects = wb_camera_recognition_get_number_of_objects(cameraL);  
    const WbCameraRecognitionObject *objects = wb_camera_recognition_get_objects(cameraL);
    for (int i = 0; i < number_of_objects; ++i)
    {
      if(strcmp(target,objects[i].model) == 0)
        if(objects[i].position_on_image[0]>UngripPosLeft && objects[i].position_on_image[0]<UngripPosRight)
          break;
          
      if(i == number_of_objects-1)
        UngripFlag = 1;
    }
    
    if((fabs(gps_values[0])<0.88||fabs(gps_values[2])<0.88) && UngripFlag)
      break;
    else
      UngripFlag = 0;
  }
 
  stop(0);
}

double JamAndWaterPos[4] = {-1.2,-1.2,  1.2,-1.2};
double CanAndBoxBPos[4] = { 1.2,-1.2,  1.2,1.2};
double BeerAndBoxPos[4] = {1.2,1.2,  -1.2,1.2};
double CanRAndBiscuitPos[4] = {-1.2,1.2,  -1.2,-1.2};

void AutoUngrip(char* target)
{  
  double floor = 0;
  double *targetPos = JamAndWaterPos;
  if(strcmp(target,"jam jar") == 0)
  {
    floor = floor2;
    targetPos = JamAndWaterPos;
    UngripPosLeft = UngripPosLeftMid;
    UngripPosRight = UngripPosRightMid;
  }
  else
  if(strcmp(target,"water bottle") == 0)
  {
    floor = floor0;
    targetPos = JamAndWaterPos;
    UngripPosLeft = UngripPosLeftMid;
    UngripPosRight = UngripPosRightMid;
  }
  else
  if(strcmp(target,"can") == 0)
  {
    floor = floor0;
    targetPos = CanAndBoxBPos;
    UngripPosLeft = UngripPosLeftSmall;
    UngripPosRight = UngripPosRightSmall;
  }
  else
  if(strcmp(target,"cereal boxB") == 0)
  {
    floor = floor1;
    targetPos = CanAndBoxBPos;
    UngripPosLeft = UngripPosLeftBig;
    UngripPosRight = UngripPosRightBig;
  }
  else
  if(strcmp(target,"beer bottle") == 0)
  {
    floor = floor0;
    targetPos = BeerAndBoxPos;
    UngripPosLeft = UngripPosLeftSmall;
    UngripPosRight = UngripPosRightSmall;
  }
  else
  if(strcmp(target,"cereal box") == 0)
  {
    floor = floor2;
    targetPos = BeerAndBoxPos;
    UngripPosLeft = UngripPosLeftBig;
    UngripPosRight = UngripPosRightBig;
  }
  else
  if(strcmp(target,"canR") == 0)
  {
    floor = floor0;
    targetPos = CanRAndBiscuitPos;
    UngripPosLeft = UngripPosLeftSmall;
    UngripPosRight = UngripPosRightSmall;
  }
  else
  if(strcmp(target,"biscuit box") == 0)
  {
    floor = floor1;
    targetPos = CanRAndBiscuitPos;
    UngripPosLeft = UngripPosLeftMid+2;
    UngripPosRight = UngripPosRightMid+2;
  }

  GoToPos(targetPos);
  targetPos+=2;
  GoToPosUngrip(targetPos,target);
  Ungrip(floor);
}


double EastPos[2] = {1.1,1.2};
double SouthPos[2] = {-1.2,1.1};
double WestPos[2] = {-1.1,-1.2};
double NorthPos[2] = {1.2,-1.1};

void Processing(char* target)
{
  DisplayMap();
  bool gripFlag = 0;
  int Direction;
  int LastDirection;
  
  const double *compass_values = wb_compass_get_values(compass); 
  const double *gps_values = wb_gps_get_values(gps);
  
  double SelfPos = atan2(gps_values[2], gps_values[0]);
  double *TargetPos;
  if(fabs(SelfPos)<0.785)
  {
    TargetPos = EastPos;
    Direction = 0;
    }
    else
      if(fabs(SelfPos)>2.356)
      {
        TargetPos = WestPos;
        Direction = 1;
        }
      else
        if(SelfPos>0)
        {
          TargetPos = SouthPos;
          Direction = 2;
          }
        else
        {
          TargetPos = NorthPos;
          Direction = 3;
          }
  LastDirection = Direction;
  double SelfAngle = atan2(compass_values[0], -compass_values[2]);
  double TargetAngel = atan2(gps_values[2]-TargetPos[1],TargetPos[0]-gps_values[0]);
  SetDirection(TargetAngel);
  
  
  while(wb_robot_step(TIME_STEP) != 1)
  {     
    double DirectionCtrl = (TargetAngel-SelfAngle)*0.3;
    if(TargetAngel>3 && SelfAngle<-3)
    {
      DirectionCtrl = -MAX_SPEED*0.3;
    }
    if(TargetAngel<-3 && SelfAngle>3)
    {
      DirectionCtrl = MAX_SPEED*0.3;
    }
    
    wb_motor_set_velocity(wheel_motors[0], MAX_SPEED - DirectionCtrl);
    wb_motor_set_velocity(wheel_motors[1], MAX_SPEED + DirectionCtrl);
    
    //printf("Using the GPS device: %.3f %.3f\n", gps_values[0], gps_values[2]); 
    compass_values = wb_compass_get_values(compass);
    gps_values = wb_gps_get_values(gps);
    
    SelfPos = atan2(gps_values[2], gps_values[0]);
    if(fabs(SelfPos)<0.785)
  {
    TargetPos = EastPos;
    Direction = 0;
    }
    else
      if(fabs(SelfPos)>2.356)
      {
        TargetPos = WestPos;
        Direction = 1;
        }
      else
        if(SelfPos>0)
        {
          TargetPos = SouthPos;
          Direction = 2;
          }
        else
        {
          TargetPos = NorthPos;
          Direction = 3;
          }
    SelfAngle = atan2(compass_values[0], -compass_values[2]);
    TargetAngel = atan2(gps_values[2]-TargetPos[1],TargetPos[0]-gps_values[0]); 
    if(LastDirection != Direction)
    {
      SetDirection(TargetAngel);
    }
    LastDirection = Direction;
    DisplayMap();
    if(oam_on)
  {
    int ds_value0 = wb_distance_sensor_get_value(so0);
   int ds_value1 = wb_distance_sensor_get_value(so1);
   int ds_value2 = wb_distance_sensor_get_value(so2);
   int ds_value3 = wb_distance_sensor_get_value(so3);
   int ds_value4 = wb_distance_sensor_get_value(so4);
   int ds_value5 = wb_distance_sensor_get_value(so5);
   int ds_value6 = wb_distance_sensor_get_value(so6);
   int ds_value7 = wb_distance_sensor_get_value(so7);
     if(ds_value0>200||ds_value1>200||ds_value2>200||ds_value3>200||
   ds_value4>200||ds_value5>200||ds_value6>200||ds_value7>200)
       {
         oam();
         compass_values = wb_compass_get_values(compass);
        gps_values = wb_gps_get_values(gps);
        
        SelfAngle = atan2(compass_values[0], -compass_values[2]);
        TargetAngel = atan2(gps_values[2]-TargetPos[1],TargetPos[0]-gps_values[0]);
        SetDirection(TargetAngel);
       }
   }
    int number_of_objects = wb_camera_recognition_get_number_of_objects(cameraR);  
    const WbCameraRecognitionObject *objects = wb_camera_recognition_get_objects(cameraR);
    for (int i = 0; i < number_of_objects; ++i)
    {
      if(strcmp(target,objects[i].model) == 0)
        if(objects[i].position_on_image[0]>30 && objects[i].position_on_image[0]<34 && objects[i].size_on_image[0]>=14)
        {
          if((strcmp(target,"biscuit box") == 0)||(strcmp(target,"cereal box") == 0)||(strcmp(target,"cereal boxB") == 0))
          {
            if(1.7*objects[i].size_on_image[0] < objects[i].size_on_image[1])
            {
              gripFlag = 1;
              break;
            }
            
          }
          else
          {
            gripFlag = 1;
            break;
          }
        }
    }
    
    if((fabs(gps_values[0])<0.6||fabs(gps_values[2])<0.6) && gripFlag)
      break;
    else
      gripFlag = 0;
  }

  stop(0);
  AutoGrip(target);
  AutoUngrip(target);
  
}
void KeyboardCtrl(int c,int pc)
{
  if ((c >= 0) && c != pc) {
      switch (c) {
        case WB_KEYBOARD_UP:
          //printf("Go forwards\n");
          moveForwards(MAX_SPEED);
          break;
        case WB_KEYBOARD_DOWN:
          //printf("Go back\n");
          moveForwards(-0.5*MAX_SPEED);
          break;
        case WB_KEYBOARD_LEFT:
          //printf("Turn left\n");
          turn(-1.5);
          break;
        case WB_KEYBOARD_RIGHT:
          //printf("Turn right\n");
          turn(1.5);
          break;
        case ' ':
          //printf("stop\n");
          stop(0.5);
          break;
        case '+':
        case 388:
        case 65585:
          //printf("Grip\n");
          AutoGrip("jam jar");
          break;
        case '-':
        case 390:
          //printf("Ungrip\n");
          //AutoUngrip(floor2);
          break;
        case 332:
        case WB_KEYBOARD_UP | WB_KEYBOARD_SHIFT:
          //printf("Increase arm height\n");
          lift(-0.4); 
          break;
        case 326:
        case WB_KEYBOARD_DOWN | WB_KEYBOARD_SHIFT:
          //printf("Decrease arm height\n");         
          lift(0);
          break;
        default:
          fprintf(stderr, "Wrong keyboard input\n");
          break;
      }
  }
  const double *gps_values = wb_gps_get_values(gps);
  printf("Using the GPS device: %.3f %.3f\n", gps_values[0], gps_values[2]); 
  DisplayMap();
}


int main() {
  initialize();
  
  while (wb_robot_step(TIME_STEP) != 1) 
   {        
    for(int i=0;i<2;i++)
    {
      Processing("biscuit box");
      Processing("can");
      //Processing("canR");
      //Processing("cereal box");
      //Processing("cereal boxB");
      Processing("jam jar");
      //Processing("water bottle");   
      Processing("beer bottle");
      
      double LastPos[4] = {-1.2,-1.2};
      GoToPos(LastPos);
      stop(2);
    }
    break;
    
    
    /*
    static int pc = 0;
    int c = wb_keyboard_get_key();
    KeyboardCtrl(c,pc);
    pc = c; 
    */
    
   }
  while (wb_robot_step(TIME_STEP) != 1) 
   {
     stop(1);
   }
  return 0;
}
