/*

Copley Controls MoveIt2 & CML Example 
Author: Anthony Redamonti

The following is a simple example of how to use MoveIt2 to plan a trajectory 
for a complex 7 axis machine and send that trajectory over a CANopen or 
EtherCAT network using Copley Controls' licensed CML C++ software.

CML can be purchased for a small one-time fee after signing a license agreement.

After following the steps below, there should be two executables running in 
seperate terminals: 
1. The MoveIt2 python demo that is a virtual robot visualized in RVIZ. The robot 
   is a ROS2 node that can accept trajectory commands.
2. A C++ program that will send trajectory commands to the end-effector of the 
   ROS2 node. After planning the trajectory, the commanded positions will be 
   extracted from the plan and streamed to a single-axis Copley Controls servo
   drive in a PVT stream. 

To run the example:

1. Install ROS2 from its debian packages and MoveIt2 from source. Follow the 
   online tutorial on the official MoveIt Humble webpage (link below). 
   https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html
   Make sure that the Colcon workspace is able to be built. 

2. Source your installation of ROS in the command line: 

   # Replace ".bash" with your shell if you're not using bash
   # Possible values are: setup.bash, setup.sh, setup.zsh
   source /opt/ros/humble/setup.bash

3. Source the Colcon workspace:

   source ~/ws_moveit2/install/setup.bash

4. Create a new package with the ROS2 command line tools:

    ros2 pkg create \
     --build-type ament_cmake \
     --dependencies moveit_ros_planning_interface rclcpp \
     --node-name hello_moveit hello_moveit

5. Open the ws_moveit2/src/hello_moveit/src/hello_moveit.cpp file and copy/paste
   this example code into it. 

6. Copy/paste the contents of the c and inc folders of CML (.h and .cpp files)
   into the following folder: ws_moveit2/src/hello_moveit/src/

7. Rebuild the Colcon workspace:

   cd ws_moveit2/
   colcon build --mixin debug ccache

8. Start the node (robot) and visualize it using RVIZ by opening a separate 
   terminal, sourcing the ROS installation setup.bash (repeat step 2), then 
   running the python MoveIt2 tutorial program with:
   
   cd ws_moveit2/
   ros2 launch moveit2_tutorials demo.launch.py

9. Start this example by opening another terminal, sourcing the ROS installation,
   sourcing the MoveIt installation, and running the following commands:

   cd ws_moveit2/
   ros2 run hello_moveit hello_moveit

Helpful Resources: 

  Your First C++ MoveIt Project: https://moveit.picknik.ai/humble/doc/tutorials/your_first_project/your_first_project.html

*/

#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>


#define PI_VALUE 	3.14159265358979323846
#define AXIS_COUNT  1
#define VIRTUAL_AXIS_COUNT  7

// Comment this out to use EtherCAT
#define USE_CAN

#include <cstdio>
#include <cstdlib>

#include "CML.h"

#if defined( USE_CAN )
#include "can_copley.h"
#elif defined( WIN32 )
#include "ecat_winudp.h"
#else
#include "ecat_linux.h"
#endif

// If a namespace has been defined in CML_Settings.h, this
// macros starts using it. 
CML_NAMESPACE_USE();

using moveit::planning_interface::MoveGroupInterface;

/* local functions */
static void showerr( const Error *err, const char *str );

/* local data */
int32 canBPS = 1000000;             // CAN network bit rate
int16 canNodeID = 1;                // CANopen node ID
uint8 timeBetweenPoints = 250;       // 20 milliseconds between points

int main(int argc, char* argv[])
{

  // The libraries define one global object of type
  // CopleyMotionLibraries named cml.
  //
  // This object has a couple handy member functions
  // including this
( LOG_EVERYTHING );

  // Create an object used to access the low level CAN network.
  // This examples assumes that we're using the Copley PCI CAN card.
#if defined( USE_CAN )
  CopleyCAN hw( "CAN0" );
  hw.SetBaud( canBPS );
#elif defined( WIN32 )
  WinUdpEcatHardware hw( "eth0" );
#else
  LinuxEcatHardware hw( "eth0" );
#endif

  // Open the network object
#if defined( USE_CAN )
  CanOpen net;
#else
  EtherCAT net;
#endif
  const Error *err = 0; 
  err = net.Open( hw );
  showerr( err, "Opening network" );

  // Initialize the amplifier using default settings
  Amp ampArr[AXIS_COUNT];
  printf( "Doing init\n" );

  AmpSettings ampSettingsObj;
  ampSettingsObj.guardTime = 0;
  ampSettingsObj.enableOnInit = false;

  err = ampArr[0].Init( net, canNodeID, ampSettingsObj );
  showerr( err, "Initting amp" );

  // array to store the counts per rev for each axis
  int countsPerRevArr[VIRTUAL_AXIS_COUNT];
  for(int i = 0; i < VIRTUAL_AXIS_COUNT; i++){
    if((i < AXIS_COUNT)){
      
      // clear latched faults
      err = ampArr[i].ClearFaults();
      showerr(err, "clearing faults");

      // enable the axis
      err = ampArr[i].Enable();
      showerr(err, "enabling axis");

      MtrInfo mtrInfoObj;
      ampArr[i].GetMtrInfo(mtrInfoObj);
      countsPerRevArr[i] = mtrInfoObj.ctsPerRev;
    }
    else{
      countsPerRevArr[i] = 131072; // made-up filler value for unused axes
    }
  }

  // Home the motor.
  HomeConfig hcfg;
  hcfg.method  = CHM_NONE;
  hcfg.velFast = 100000;
  hcfg.velSlow = 50000;
  hcfg.accel   = 90000;
  hcfg.offset  = 0;

  // Send the command to home each motor
  for( int i = 0; i < AXIS_COUNT; i++ ){
    err = ampArr[i].GoHome( hcfg );
    showerr( err, "Going home" );
    err = ampArr[i].WaitMoveDone( 20000 ); 
    showerr( err, "waiting on home" );
  }

  Linkage linkageObj;
  err = linkageObj.Init(AXIS_COUNT, ampArr);
  showerr( err, "initializing linkage object" );

  err = linkageObj.SetMoveLimits(2000, 2000, 2000, 5000);
  showerr( err, "setting linkage object move limits" );

  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");
  
  // Create a planning group
  static const std::string PLANNING_GROUP = "panda_arm";

  // Create the MoveIt MoveGroup Interface
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");

  // uncomment these two lines to increase the speed of the PVT stream (increase the spacing between points). Default is 0.01.
  //move_group_interface.setMaxVelocityScalingFactor(0.05);
  //move_group_interface.setMaxAccelerationScalingFactor(0.05);

  bool success = true;

  while(1) {

    // create a target pose for the robot
    geometry_msgs::msg::Pose target_pose;
      target_pose.orientation.w = 1.0;
      target_pose.position.x = 0.2;
      target_pose.position.y = 0.2;
      target_pose.position.z = 0.5;

    move_group_interface.setPoseTarget(target_pose);

    // create a plan to the target pose
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // Execute the plan
    if (success)
    {
      PvtConstAccelTrj pvtConstAccelTrjObj;
      err = pvtConstAccelTrjObj.Init(AXIS_COUNT);
      showerr(err, "initializing the PvtConstAccelTrj object");

      // get the trajectory from the plan
      moveit_msgs::msg::RobotTrajectory msg = my_plan.trajectory_;

      std::vector<int>::size_type totalNumPvtPoints = msg.joint_trajectory.points.size();
      printf("\nThe total number of PVT points = %ld", totalNumPvtPoints);

      // move the linkage to the starting position of the PVT stream 
      Point<AXIS_COUNT> startingPos;

      // extract all PVT data
      for (unsigned i=0; i<totalNumPvtPoints; i++)
      {
        printf("\nPVT point = %d", i);
        std::vector<double> pvtPoint; // we only have a single axes linkage for now

        // get the size (how many axes are in PVT point)
        std::vector<int>::size_type size2 = msg.joint_trajectory.points[i].positions.size();

        for (unsigned j=0; j<size2; j++)
        {
          float radFloat = msg.joint_trajectory.points[i].positions[j];
          printf("\nAxis %d radians: %g", j, radFloat);
          float degrees = (180/PI_VALUE) * radFloat; // convert radians to degrees
          printf(" degrees: %g", degrees);
          degrees = degrees / 360.0; // 360 degrees in one rev
          double counts = degrees * (countsPerRevArr[j] * 1.0); // convert degrees to encoder counts
          printf(" counts: %f", counts);

          // in our case, we only are using 1 axes, so form a 1 axes PVT point
          if(j < AXIS_COUNT){
            
              pvtPoint.push_back(counts);

            // record the starting position
            if(i == 0){
              startingPos[j] = counts;
            }
          }
        }

        // add the PVT point to the PVT constant acceleration object
        err = pvtConstAccelTrjObj.addPvtPoint(&pvtPoint, &timeBetweenPoints);
        showerr( err, "adding PVT point to pvtConstAccelTrjObj" );
      }

      // print the starting position
      for(int i = 0; i < AXIS_COUNT; i++){
        printf("\nAxis %d Starting Position: %f", i, startingPos[i]);
      }

      err = linkageObj.MoveTo(startingPos);
      showerr(err, "linkageObj.MoveTo(startingPos)");

      err = linkageObj.WaitMoveDone(-1);
      showerr(err, "waiting for linkage to move to starting position");

      printf("\nLinkage successfully moved to starting position.");

      err = linkageObj.SendTrajectory(pvtConstAccelTrjObj);
      showerr(err, "sending PVT const accel trajectory to linkage object");

      printf("\nPVT stream initiated.");

      //move_group_interface.execute(my_plan);

      err = linkageObj.WaitMoveDone(-1);
      showerr(err, "waiting for PVT move to finish");

      printf("\nPVT move successfully completed.");
    }
    else
    {
      RCLCPP_ERROR(logger, "Planning failed!");
      
      //Shutdown ROS
      rclcpp::shutdown();
      
      return -1;
    }
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}

/**************************************************/

static void showerr( const Error *err, const char *str )
{
   if( err )
   {
      printf( "Error %s: %s\n", str, err->toString() );
      exit(1);
   }
}
