/*

CombineScurveMoves.cpp

The following example demonstrates how to combine two 
s-curve moves using PVT streaming. 

*/

// Comment this out to use EtherCAT
#define USE_CAN

#include <cstdio>
#include <cstdlib>
#include <iostream>

#include "CML.h"

#if defined( USE_CAN )
#include "can/can_copley.h"
#elif defined( WIN32 )
#include "ecat/ecat_winudp.h"
#else
#include "ecat/ecat_linux.h"
#endif

using std::cout;
using std::endl;

// If a namespace has been defined in CML_Settings.h, this
// macros starts using it. 
CML_NAMESPACE_USE();

/* local functions */
static void showerr(const Error* err, const char* str);

/* local defines */
#define AMPCT 3

/* local data */
int32 canBPS = 1000000;                // CAN network bit rate
const char* canDevice = "CAN0";        // Identifies the CAN device, if necessary
int16 canNodeID = 1;                   // CANopen node ID of first amp.  Second will be ID+1, etc.

/// <summary>
/// Load the s-curve data into the PVT object.
/// </summary>
/// <param name="linkScurveObj">The linkage s-curve calculator</param>
/// <param name="positions">The positions to move to for each axis.</param>
/// <param name="times">The time values for each PVT point.</param>
void ExtractTrajectoryFromScurveObject(LinkTrjScurve& linkScurveObj, vector<vector<double>>* positions, vector<uint8>* times)
{
    (*positions).clear();
    (*times).clear();

    uint8 timeConstant = 10; // does not matter what this is initially set to. It will be overwritten by NextSegment method.

    while (timeConstant != 0)
    {
        double posTemp[AMPCT];
        double velTemp[AMPCT];

        const Error* err = linkScurveObj.NextSegment(posTemp, velTemp, timeConstant);
        showerr(err, "retrieving the next segment from the s-curve calculator");

        vector<double> posTempVec;
        vector<double> velTempVec;

        for (int i = 0; i < AMPCT; i++)
        {
            posTempVec.push_back(posTemp[i]);
            velTempVec.push_back(velTemp[i]);
        }

        (*positions).push_back(posTempVec);
        (*times).push_back(timeConstant);
    }
}

/// <summary>
/// Load the positions and times into the PvtConstAccelTrj object. The PvtConstAccelTrj class
/// will automatically calculate the velocity based of the position and time values entered.
/// </summary>
/// <param name="positionsIn">Positions to load.</param>
/// <param name="timesIn">Time values. Each time represents the interpolation time value.</param>
/// <param name="pvtObjIn">The PVT object that stores the trajectory.</param>
void LoadPointsIntoPvtObj(vector<vector<double>>& positionsIn, vector<uint8>& timesIn, PvtConstAccelTrj& pvtObjIn) 
{
    int count = 0;

    // load the PVT points into the PvtConstAccelTrl object
    while (count < (int)positionsIn.size())
    {
        vector<double>posTempVec;
        for (int j = 0; j < AMPCT; j++)
        {
            posTempVec.push_back(positionsIn[count][j]);
        }

        uint8 timeConstant = timesIn[count];

        const Error* err = pvtObjIn.addPvtPoint(&posTempVec, &timeConstant);
        showerr(err, "adding PVT point to PVT object");

        count++;
    }
}

int main(void)
{
    // The libraries define one global object of type
    // CopleyMotionLibraries named cml.
    //
    // This object has a couple handy member functions
    // including this one which enables the generation of
    // a log file for debugging
    cml.SetDebugLevel(LOG_DEBUG);

    // Create an object used to access the low level CAN network.
    // This examples assumes that we're using the Copley PCI CAN card.
#if defined( USE_CAN )
    CopleyCAN hw("CAN0");
    hw.SetBaud(canBPS);
#elif defined( WIN32 )
    WinUdpEcatHardware hw("eth0");
#else
    LinuxEcatHardware hw("eth0");
#endif

    // Open the network object
#if defined( USE_CAN )
    CanOpen net;
#else
    EtherCAT net;
#endif

    const Error* err = NULL;

    err = net.Open(hw);
    showerr(err, "Opening network");

    // Initialize the amplifiers using default settings
    Amp amp[AMPCT];
    printf("Doing init\n");
    for (int i = 0; i < AMPCT; i++)
    {
        printf("Initing %d\n", canNodeID + i);
        err = amp[i].Init(net, canNodeID + i);
        showerr(err, "Initting amp");
    }

    // Set the position to zero on all axes. Just used for testing this example. Remove in production.
    for (int i = 0; i < AMPCT; i++)
    {
        amp[i].SetPositionActual(0);
    }

    // Create a linkage object holding these amps
    Linkage link;
    err = link.Init(AMPCT, amp);
    showerr(err, "Linkage init");

    PvtConstAccelTrj pvtObj;
    err = pvtObj.Init(AMPCT);
    showerr(err, "initializing pvt object");

    double velocity = 10000; // firmware units = 0.1 counts/sec
    double accel = 10000;    //                = 10 counts/sec^2
    double decel = 10000;    //                = 10 counts/sec^2
    double jerk = 100000;    //                = 100 counts/sec^3

    // Setup the velocity, acceleration, deceleration & jerk limits
    // for multi-axis moves using the linkage object
    err = link.SetMoveLimits(velocity, accel, decel, jerk);
    showerr(err, "setting move limits");

    Point<AMPCT> startPos;
    err = link.GetPositionCommand(startPos);
    showerr(err, "getting the commanded position");

    // Create an arbitrary N dimensional target position.
    Point<AMPCT> targetPosition;
    for (int i = 0; i < AMPCT; i++)
    {
        targetPosition[i] = 1000; // move 1,000 encoder counts on each axis
    }

    LinkTrjScurve firstScurveMove;

    err = firstScurveMove.Calculate(startPos, targetPosition, velocity, accel, decel, jerk);
    showerr(err, "calculating trajectory");

    err = firstScurveMove.StartNew();
    showerr(err, "starting trajectory");

    vector<vector<double>> positionsVecMove1;
    vector<uint8> timesVec1;

    // extract all positions and times from the first s-curve move
    ExtractTrajectoryFromScurveObject(firstScurveMove, &positionsVecMove1, &timesVec1);

    //// print the original points of the s-curve move to the console
    //for (int i = 0; i < (int)positionsVecMove1[0].size(); i++)
    //{
    //    for (int j = 0; j < (int)positionsVecMove1.size(); j++) 
    //    {
    //        cout << (double)positionsVecMove1[j][i] << endl;
    //    }
    //    cout << "Next Amp" << endl;
    //}

    LinkTrjScurve secondScurveMove;

    err = secondScurveMove.Calculate(targetPosition, startPos, velocity, accel, decel, jerk);
    showerr(err, "calculating trajectory");

    err = secondScurveMove.StartNew();
    showerr(err, "starting trajectory");

    vector<vector<double>> positionsVecMove2;
    vector<uint8> timesVec2;

    // extract all positions and times from the first s-curve move
    ExtractTrajectoryFromScurveObject(secondScurveMove, &positionsVecMove2, &timesVec2);

    // Because we want the moves to seamlessly transition from one to the other, 
    // the last point can be erased in the first move. Essentially, our ending 
    // position for the first move is the same as the starting position for the 
    // second move, so there is no reason to dwell there unnecessarily. 
    positionsVecMove1.erase(positionsVecMove1.end() - 1);
    timesVec1.erase(timesVec1.end() - 1);

    LoadPointsIntoPvtObj(positionsVecMove1, timesVec1, pvtObj);

    LoadPointsIntoPvtObj(positionsVecMove2, timesVec2, pvtObj);

    err = link.SendTrajectory(pvtObj);
    showerr(err, "starting the move");

    err = link.WaitMoveDone(-1);
    showerr(err, "waiting for move to finish");

    return 0;
}

/**************************************************/

static void showerr(const Error* err, const char* str)
{
    if (err)
    {
        printf("Error %s: %s\n", str, err->toString());
        exit(1);
    }

}
