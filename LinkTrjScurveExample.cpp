/*

LinkTrjScurveExample.cpp

This is an example of how to use a LinkTrjScurve class to calculate a linkage
trajectory and extract the trajectory data. The data can be altered and then
streamed in a PVT stream using the PvtConstAccelTrj class.

The s-curve trajectory in this example is altered on the fly during the move
by doubling the time value in the PVT stream.

Below are the steps performed in this example:

1) The LinkTrjScurve class is used to calculate positions and times for
the s-curve move.

2) The position data is injected with more position data to provide more
resolution for the stream. We want the list of position data to be roughly 20
milliseconds apart.

3) The position data is smoothed 100 times using a simple smoothing algorithm.

4) The PVT data is loaded into the PvtConstAccelTrj class, and the linkage move
is started.

5) At the halfway point, the time value is altered dynamically in the PVT stream.

6) Link.WaitMoveDone will wait for the move to complete.

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

const int softwareBufferMaxPoints = 50; // max size of the PVT buffer in CML (software)
const int hardwareBufferMaxPoints = 64; // max size of the PVT buffer in the drive (hardware)

/// <summary>
/// Smooth the position data.
/// </summary>
/// <param name="vecIn"></param>
void SmoothPositionProfile(vector<vector<double>>& vecIn)
{
    for (int i = 1; i < (int)vecIn.size() - 1; i++)
    {
        for (int j = 0; j < (int)vecIn[0].size(); j++)
        {
            double diff1 = vecIn[i][j] - vecIn[i - 1][j];
            double diff2 = vecIn[i + 1][j] - vecIn[i][j];

            double diff = diff2 - diff1;
            vecIn[i][j] = (diff / 2.0) + vecIn[i][j];
        }
    }
}

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
/// Update the linkage trajectory s-curve object with the user units of each amp object
/// </summary>
/// <param name="linkTrjScurveIn">The linkage trajector s-curve object</param>
/// <param name="ampArr">The array of amplifier objects</param>
/// <param name="ampCount">The number of amplifiers in the array</param>
/// <returns></returns>
void UpdateLinkTrjScurveUserUnits(LinkTrjScurve& linkTrjScurveIn, Amp* ampArr, int ampCount) 
{
    const int ampCountTemp = ampCount;
    uunit* u2lPosArr = new uunit[ampCountTemp];
    uunit* u2lVelArr = new uunit[ampCountTemp];

    // get the latest user-units from each amplifier in the linkage
    for (int i = 0; i < ampCountTemp; i++) 
    {
        u2lPosArr[i] = ampArr[i].PosUser2Load(1);   // 1 counts units
        u2lVelArr[i] = ampArr[i].VelUser2Load(0.1); // 0.1 counts/sec units
    }

    // provide the user units to the LinkTrjScurve class
    linkTrjScurveIn.UpdateUserToLoadUnitConverters(u2lPosArr, u2lVelArr);

    delete[] u2lPosArr;
    delete[] u2lVelArr;
}

/// <summary>
/// Provide more points for the PVT stream by filling in the altered positions and times using the inputs positions and times. 
/// Basically, if there is a PVT point with a large time value, we can replace it with multiple PVT points that use a smaller
/// time value. This will allow us to adjust the time values of the PVT stream to either speed up or slow down during the move.
/// </summary>
/// <param name="positionsVecIn">Input positions vector.</param>
/// <param name="timesVecIn">Input times vector.</param>
/// <param name="alteredPositionsVec">The output positions vector.</param>
/// <param name="alteredTimesVec">The output times vector.</param>
/// <param name="alteredTimesConstant">The altered time constant. Something relatively small, like 10 milliseconds.</param>
void CreateAlteredPositionsAndTimesVectors(vector<vector<double>>& positionsVecIn, vector<uint8>& timesVecIn, vector<vector<double>>& alteredPositionsVec, vector<uint8>& alteredTimesVec, uint8 alteredTimeConstantIn) 
{
    // break the trajectory into 20ms segments
    for (int i = 0; i < (int)positionsVecIn.size(); i++)
    {
        // append the original position
        alteredPositionsVec.push_back(positionsVecIn[i]);

        // if the time is zero, it's the last PVT point. Use the altered time constant.
        if (timesVecIn[i] == 0)
        {
            alteredTimesVec.push_back(alteredTimeConstantIn);
            continue;
        }
        // if the time is non-zero but less than the altered time constant, use it and continue (do not inject any additional positions).
        else if (timesVecIn[i] < alteredTimeConstantIn)
        {
            alteredTimesVec.push_back(timesVecIn[i]);
            continue;
        }
        // the time value is larger than the altered time value. Therefore, we can inject smaller positions into the trajectory.
        else
        {
            alteredTimesVec.push_back(alteredTimeConstantIn);
        }

        // do not inject any positions if this is the last position (target)
        if (i < (int)positionsVecIn.size() - 1)
        {
            // the number of points to inject 
            int injectionFactor = (timesVecIn[i] / alteredTimeConstantIn) - 1;

            for (int t = 1; t <= injectionFactor; t++)
            {
                vector<double>posTempVec2;
                for (int j = 0; j < AMPCT; j++)
                {
                    // spacing between position injections
                    double deltaPos = (positionsVecIn[i + 1][j] - positionsVecIn[i][j]) / (1.0 + injectionFactor);
                    double newPos = positionsVecIn[i][j] + (deltaPos * t);
                    posTempVec2.push_back(newPos);
                }

                alteredPositionsVec.push_back(posTempVec2);
                alteredTimesVec.push_back(alteredTimeConstantIn);
            }
        }
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

        MtrInfo mtrInfo;
        err = amp[i].GetMtrInfo(mtrInfo);
        showerr(err, "Getting motor info\n");

        // configuring non-default user units. user unit value of 1.0 will be equal to 1 motor rev.
        err = amp[i].SetCountsPerUnit(mtrInfo.ctsPerRev);
        showerr(err, "Setting cpr\n");
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

    // set the number of points to keep in the drive's buffer
    pvtObj.maxBufferPoints = hardwareBufferMaxPoints;

    double velocity = 0.5; // this is the max velocity achievable for the system in terms of user units. 
    double accel = 1.0;
    double decel = 1.0;
    double jerk = 10.0;

    // Setup the velocity, acceleration, deceleration & jerk limits
    // for multi-axis moves using the linkage object
    err = link.SetMoveLimits(velocity, accel, decel, jerk);
    showerr(err, "setting move limits");

    Point<AMPCT> startPos;
    err = link.GetPositionCommand(startPos);
    if (err) return -1;

    // Create an arbitrary N dimensional target position.
    Point<AMPCT> targetPosition;
    for (int i = 0; i < AMPCT; i++)
    {
        targetPosition[i] = 0.5; // one-half of a motor revolution
    }

    LinkTrjScurve linkTrjScurveObj;

    err = linkTrjScurveObj.Calculate(startPos, targetPosition, velocity, accel, decel, jerk);
    showerr(err, "calculating trajectory");

    err = linkTrjScurveObj.StartNew();
    showerr(err, "starting trajectory");

    UpdateLinkTrjScurveUserUnits(linkTrjScurveObj, amp, AMPCT);

    vector<vector<double>> positionsVec;
    vector<uint8> timesVec;

    // reads all positions and times from s-curve object
    ExtractTrajectoryFromScurveObject(linkTrjScurveObj, &positionsVec, &timesVec);

    //// print the original points of the s-curve move to the console
    //for (int i = 0; i < (int)positionsVec[0].size(); i++)
    //{
    //    for (int j = 0; j < (int)positionsVec.size(); j++) 
    //    {
    //        cout << (double)positionsVec[j][i] << endl;
    //    }

    //    cout << "Next Amp" << endl;
    //}

    uint8 alteredTimeConstant = 20; // 20 millisecond chunks

    vector<vector<double>> alteredPositionsVec;
    vector<uint8> alteredTimesVec;

    CreateAlteredPositionsAndTimesVectors(positionsVec, timesVec, alteredPositionsVec, alteredTimesVec, alteredTimeConstant);

    //// add the last position (target) again
    //alteredPositionsVec.push_back(positionsVec[positionsVec.size() - 1]);

    //// set the last time to zero (end of move)
    //alteredTimesVec.push_back(0);

    // smooth the profile 10 times. 
    for (int i = 0; i < 10; i++)
    {
        SmoothPositionProfile(alteredPositionsVec);
    }

    //// print the altered points of the s-curve move to the console
    //for (int i = 0; i < (int)alteredPositionsVec[0].size(); i++)
    //{
    //    for (int j = 0; j < (int)alteredPositionsVec.size(); j++)
    //    {
    //        cout << (double)alteredPositionsVec[j][i] << endl;
    //    }

    //    cout << "Altered Next Amp" << endl;
    //}

    int count = 0;
    int halfwayPoint = (int)alteredPositionsVec.size() / 2;

    // load the first few points and start the move
    while (((int)pvtObj.getNumberOfPvtPoints() < softwareBufferMaxPoints) && (count < (int)alteredPositionsVec.size()))
    {
        vector<double>posTempVec;
        for (int j = 0; j < AMPCT; j++)
        {
            posTempVec.push_back(alteredPositionsVec[count][j]);
        }

        uint8 timeConstant = alteredTimesVec[count];

        err = pvtObj.addPvtPoint(&posTempVec, &timeConstant);
        showerr(err, "adding PVT point to PVT object");

        count++;
    }

    err = link.SendTrajectory(pvtObj);
    showerr(err, "starting the move");

    // stream the rest of the PVT data
    while (count < (int)alteredPositionsVec.size())
    {
        // keep the number of points in the software buffer to an acceptable minimum
        if ((int)pvtObj.getNumberOfPvtPoints() < softwareBufferMaxPoints)
        {
            vector<double>posTempVec;
            for (int j = 0; j < AMPCT; j++)
            {
                posTempVec.push_back(alteredPositionsVec[count][j]);
            }

            uint8 timeConstant = alteredTimesVec[count];

            // slow down the move (alter it on the fly)
            if (count >= halfwayPoint)
            {
                timeConstant *= 2;
            }

            err = pvtObj.addPvtPoint(&posTempVec, &timeConstant);
            showerr(err, "adding PVT point to PVT object");

            count++;
        }
    }

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
