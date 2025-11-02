// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <functional>
#include <memory>
#include <cstdio>
#include <cstdlib>
#include "CML.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Comment this out to use EtherCAT
#define USE_CAN

#if defined( USE_CAN )
#include "can_copley.h"
#elif defined( WIN32 )
#include "ecat_winudp.h"
#else
#include "ecat_linux.h"
#endif

using std::atoi;
using std::placeholders::_1;

// If a namespace has been defined in CML_Settings.h, this
// macros starts using it. 
CML_NAMESPACE_USE();

/* local functions */
static void showerr( const Error *err, const char *str );

int commandedPosition = 0;
Amp amp;  // An instance of the Amp class (servo drive)

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }
  
private:
  
  void topic_callback(const std_msgs::msg::String & msg) //const
  {
    commandedPosition = atoi(msg.data.c_str());
    //printf("commanded position: %d\n", commandedPosition);
    RCLCPP_INFO(this->get_logger(), "New commanded position received: '%d'", commandedPosition);
    
    const Error* err = amp.DoMove(commandedPosition * 100); // units = encoder counts
    showerr(err, "moving to new position");
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

static void showerr( const Error *err, const char *str )
{
   if( err )
   {
      printf( "Error %s: %s\n", str, err->toString() );
      exit(1);
   }
}

int main(int argc, char * argv[])
{
  int16 canNodeID = 1;      // CAN node ID of this servo drive
  int32 canBPS = 1000000;   // CAN network bit rate
  
  // Create an object used to access the low level CAN network.
  // This examples assumes that we're using one of the Copley CAN cards.
#if defined( USE_CAN )
  CopleyCAN hw( "CAN0" );
  hw.SetBaud( canBPS );
#elif defined( WIN32 )
  WinUdpEcatHardware hw( "eth0" );
#else
  LinuxEcatHardware hw( "eth0" );
#endif
  
#if defined( USE_CAN )
  CanOpen net;
#else
  EtherCAT net;
#endif
  
  // Open the network object
  const Error* err = net.Open( hw );
  showerr( err, "Opening network" );
  
  // Initialize the amplifier using default settings
  err = amp.Init( net, canNodeID );
  showerr( err, "Initting amp" );
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
