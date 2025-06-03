/*

The following example is called Robot Teach Mode and is part of
a YouTube video on the Copley Controls YouTube channel.

Application: The user physically moves a robot by hand to teach
the robot the desired motion profile. While in "teach mode," the
robot is disabled and simply recording its positions over time.
When the user is done teaching, they will press a button and the 
robot will repeat the same motion profile that it was taught 
during teach mode.

CML uses TxPDO's on an EtherCAT network to synchronously record 
position, current, and digital input data on three nodes.

After the TxPDO's have been initialized, the program will wait 
for the user to press any key to begin teach mode. When teach mode
begins, the positions on all three axes are being recorded and 
transferred to a PvtConstAccelTrj object. 

When the user is done teaching, they will press any key to end 
teach mode. The PvtConstAccelTrj object will cease recording
position data and will be passed as an argument to the linkage
object for execution.

*/

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <list>
#include <mutex>
#include "CML.h"
#include "ecat/ecat_winudp.h"

using std::cout;
using std::endl;
using std::mutex;
using std::vector;
using std::list;

// This define is used below to set the move speed for the example moves in 
// encoder count / sec units.  Set it to something reasonable for your system.
#define MOVE_SPEED      40000

// If a namespace has been defined in CML_Settings.h, this
// macros starts using it. 
CML_NAMESPACE_USE();

/* local functions */
static void showerr( const Error *err, const char *str );
void appendLastPosition(vector<double>& vector, size_t number);

#define numberOfAxes 2

// Define a class based on the TPDO base class.
// A Transmit PDO is one that's transmitted by the
// CANopen node and received by the master.
// This PDO will be used to send the motor position,
// actual current, and digital input states from the
// drive to the master.
class TpdoActPosActCurrent: public TPDO
{

public:

    // a boolean variable for when we are teaching the axis
    bool isTeaching;

    // a vector of position data that was recorded when in teach mode.
    vector<double> positionsVector;

   // Default constructor does nothing
   TpdoActPosActCurrent(){}

   // Called once at startup to map the PDO and configure CML to 
   // listen for it.
   const Error *Init( Amp &ampObj, int slot);

   // This function is called when the PDO is received
   virtual void Received(void);

private:

    // a mutex for acting on the received data
    mutex tpdoMutex;

    // These variables are used to map objects to the PDO.
    // Different mapping objects are used for different data sizes.
    // Position is a 32-bit value.
    Pmap32 actualPosition;
    Pmap32 digitalInputs;
    Pmap16 actualCurrent;
};

/* local data */
int32 canBPS = 1000000;             // CAN network bit rate (1MB)
Amp ampArray[numberOfAxes];
TpdoActPosActCurrent tpdo[numberOfAxes];

/**************************************************
* Just home the motor and do a bunch of random
* moves.
**************************************************/
int main( void )
{
    uint8 timeBetweenPoints{ 15 }; // 15 milliseconds between points

    // The libraries define one global object of type
    // CopleyMotionLibraries named cml.
    //
    // This object has a couple handy member functions
    // including this one which enables the generation of
    // a log file for debugging
    //cml.SetDebugLevel( LOG_EVERYTHING );
    //cml.SetFlushLog( true );

    // Open the CANopen network object
    //CanOpen canOpen; //type , variable name
    WinUdpEcatHardware hw("192.168.0.98");//first adapter to respond 
    EtherCAT net;
    const Error* err = net.Open(hw);
    showerr( err, "Opening EtherCAT network" );

    // I'll use some custom amplifier settings, in particular
    // I'll use a SYNC period of 11ms rather then the default 10ms
    AmpSettings settings;
    settings.synchPeriod = 10000; // 10,000 = SYNC pulse sent every 10 milliseconds.
 
    int node = -1;

    printf("\nDoing init\n");
    
    // Step 1: initialize the nodes on the EtherCAT network
    // Step 2: set the nodes to pre-op mode.
    // Step 3: initialize the TxPDO's.
    // Step 4: start the nodes on the network.
    for (int i = 0; i < numberOfAxes; i++)
    {
       err = ampArray[i].Init(net, node * (i + 1), settings); 
       
       std::this_thread::sleep_for(std::chrono::seconds(1));
       

       while (err) {
           cout << "Node " << i + 1 << " failed to init. Going to try again." << endl;
           err = ampArray[i].Init(net, node * (i + 1), settings);
           std::this_thread::sleep_for(std::chrono::seconds(1));
       }
       
       //showerr(err, "Initting amp");

       err = ampArray[i].PreOpNode();
       
       while (err) {
           //cout << "Node " << i + 1 << " failed to pre-op. Going to try again." << endl;
           std::this_thread::sleep_for(std::chrono::seconds(1));
           err = ampArray[i].PreOpNode();
       }

       //showerr(err, "Preopping node");

       err = tpdo[i].Init(ampArray[i], 2);
       
       while (err) {
           //cout << "TxPDO " << i + 1 << " failed to initialize. Going to try again." << endl;
           std::this_thread::sleep_for(std::chrono::seconds(1));
           err = tpdo[i].Init(ampArray[i], 2);
       }

       //showerr(err, "Initting tpdo");

       err = ampArray[i].StartNode();
       
       while (err) {
           //cout << "Node " << i + 1 << " failed to start. Going to try again." << endl;
           std::this_thread::sleep_for(std::chrono::seconds(1));
           err = ampArray[i].StartNode();
       }

       //showerr(err, "Starting node");
    }

    // Create a linkage object
    Linkage linkageObj;
    err = linkageObj.Init(numberOfAxes, ampArray);
    showerr(err, "Linkage init");  

    cout << "The TPDO's have been initialized." << endl;

    cout << "Press any key to begin teaching mode" << endl;

    getchar();

    for (int i = 0; i < numberOfAxes; i++) {
        ampArray[i].Disable();
    }

    for (int i = 0; i < numberOfAxes; i++) {
        tpdo[i].isTeaching = true;
    }

    cout << "The positions are being recorded. Press any key to stop teaching." << endl;

    getchar();
    
    for (int i = 0; i < numberOfAxes; i++) {
        tpdo[i].isTeaching = false;
    }

    cout << "Recording stopped. Move will now begin." << endl;

    for (int i = 0; i < numberOfAxes; i++) {
        err = ampArray[i].Enable();
        while (err) {
            cout << "Error occurred re-enabling node " << i + 1 << endl;

            // if err == timeout, reinit the node on the network.
            if (err->GetID() == 124) {

                cout << "Node " << i + 1 << " timed out. Attempting reinitialization." << endl;

                err = ampArray[i].ReInit();
                if (err) {
                    showerr(err, "Reinitializing node");
                }
                else {
                    cout << "Node " << i + 1 << " has been reinitialized." << endl;
                }
            }

            err = ampArray[i].Enable();
        }
    }

    // Find the axis that collected the most position data.
    int axisWithMostPosData{ 0 };
    for (int i = 0; i < numberOfAxes; i++) {
        if (tpdo[axisWithMostPosData].positionsVector.size() > tpdo[i].positionsVector.size()) {
            axisWithMostPosData = i;
        }
    }

    // make the vectors of position data equal length.
    size_t sizeForAllAxes{ tpdo[axisWithMostPosData].positionsVector.size() };
    for (int i = 0; i < numberOfAxes; i++) {
        int difference = sizeForAllAxes - tpdo[i].positionsVector.size();
        if (difference > 0) {
            appendLastPosition(tpdo[i].positionsVector, difference);
        }
    }

    // load the PVT points using 15 ms between points.
    PvtConstAccelTrj pvtConstAccelTrjObj;
       
    // initialize it with the number of axes.
    pvtConstAccelTrjObj.Init(numberOfAxes);
       
    // load all the PVT points into the PVT object.
    for (size_t i = 0; i < sizeForAllAxes; i++) {
        vector<double> singlePointVec;

        for (int j = 0; j < numberOfAxes; j++) {
            singlePointVec.push_back(tpdo[j].positionsVector[i]);
        }

        pvtConstAccelTrjObj.addPvtPoint(&singlePointVec, &timeBetweenPoints); // load into PVT object.
    }

    Point<numberOfAxes> startingPosition;

    // set some reasonable move constraints
    double velLimit = MOVE_SPEED;            // velocity (encoder counts / second)
    double accelLimit = MOVE_SPEED * 10;     // acceleration (cts/sec/sec)
    double decelLimit = MOVE_SPEED * 10;     // deceleration (cts/sec/sec)
    double jerkLimit = MOVE_SPEED * 50;      // jerk (cts/sec/sec/sec)
    err = linkageObj.SetMoveLimits(velLimit, accelLimit, decelLimit, jerkLimit);
    showerr(err, "Setting linkage move limits");


    // retrieve the starting position for the PVT move.
    list<double>::iterator iterList;
    vector<list<double>>* posVecPntr = pvtConstAccelTrjObj.getPositionsPntr();
    for (int i = 0; i < numberOfAxes; i++) {
        iterList = (*posVecPntr)[i].begin();
        startingPosition[i] = *iterList;
    }

    // move to the starting position before beginning the PVT move.
    err = linkageObj.MoveTo(startingPosition);
    showerr(err, "Moving to starting position");

    // wait for the move to finish.
    linkageObj.WaitMoveDone(-1);
    showerr(err, "Waiting for move to starting position to finish");

    // send the PVT points (trajectory) to the linkage object.
    err = linkageObj.SendTrajectory(pvtConstAccelTrjObj, true); // true = start the move immediately.
    showerr(err, "Starting the linkage move");

    // Wait for the move to finish. Set to -1 to wait indefinitely.
    err = linkageObj.WaitMoveDone(-1);
    showerr(err, "Waiting for the move to complete");

    return 0;
}

/**************************************************/

static void showerr( const Error *err, const char *str )
{
   if( err )
   {
      printf( "Error %s: %s\n", str, err->toString() );
	  getchar();
      exit(1);
   }
}

/**
 * Transmit PDO handling.
 * Each amplifier is being configured to send out a CAN message
 * once every SYNC period.  This class handles receiving these
 * PDOs.
 *
 * @param ampObj The amplifier to map this TxPDO on.
 * @param slotNumber  The slot number used by this TxPDO.
 * @return NULL on success, or an error object on failure.
 */
const Error *TpdoActPosActCurrent::Init( Amp &ampObj, int slotNumber)
{
    const Error* err{ NULL };

    // set the isTeaching boolean variable to false.
    isTeaching = false;

    // Initialize the transmit PDO
    err = TPDO::Init(0x280 + slotNumber * 0x100 + ampObj.GetNodeID());

    // Init the position variable using the CANopen object ID 
    if (!err) err = actualPosition.Init(OBJID_POS_LOAD, 0); //   32 bits
    if (!err) err = digitalInputs.Init(0x60FD);             //   32 bits
    if (!err) err = actualCurrent.Init(0x221C);             // + 16 bits
                                                            // ---------
                                                            //   80 bits
    // Add the mapped variable
    if (!err) err = AddVar(actualPosition);
    if (!err) err = AddVar(digitalInputs);
    if (!err) err = AddVar(actualCurrent);

    // Program this PDO in the amp, and enable it
    if (!err) err = ampObj.PdoSet(slotNumber, *this);

   return err;
}

/**
 * This function will be called by the high priority CANopen receive
 * thread when the PDO is received.  
 *
 * By the time this function is called, the data from the two mapped objects will
 * have been parsed from the input message.  It can be accessed by the Pmap objects
 * that we created.
 *
 * Keep in mind that this function is called from the same thread that receives all
 * CANopen messages.  Keep any processing here short and don't try to do any SDO access.
 * Often it's best to simply post a semaphore here and have another thread handle the data.
 *
 */
void TpdoActPosActCurrent::Received( void )
{
   //printf( "PDO received - position: %-9d current: %-5d \r", actualPosition.Read(), actualCurrent.Read() );

   if (isTeaching) {
       
       // protect the access to the pvt object.
       tpdoMutex.lock();

       // push the new position on the vector
       positionsVector.push_back(actualPosition.Read()); // the position read from the drive.
       
       // unlock the mutex.
       tpdoMutex.unlock();
   }

}

// Append the position vector's last element.
// The number of append operations = number.
void appendLastPosition(vector<double>& posVector, size_t number) {
    double lastPosition = posVector[posVector.size() - 1];
    for (size_t i = 0; i < number; i++) {
        posVector.push_back(lastPosition);
    }
}
