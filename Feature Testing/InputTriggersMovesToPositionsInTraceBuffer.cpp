/*

InputTriggersMovesToPositionsInTraceBuffer.cpp

Below is an example of how to load positions into the drive's
trace buffer and use an input(s) to trigger the move to each
position. 

CVM Registers are configured as:
(1) offset into table
(2) size of table
(3) head pointer
(4) tail pointer

IN1 triggers Axis A. IN2 triggers Axis B. IN3 triggers Axis C. 
Wire all three inputs together to move all three axes at the same time. 

*/

// Comment this out to use EtherCAT
//#define USE_CAN

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include "CML.h"

using std::cout;
using std::endl;

#if defined( USE_CAN )
#include "can/can_copley.h"
#elif defined( WIN32 )
#include "ecat/ecat_winudp.h"
#else
#include "ecat/ecat_linux.h"
#endif

// If a namespace has been defined in CML_Settings.h, this
// macros starts using it. 
CML_NAMESPACE_USE();

/* local functions */
static void showerr(const Error* err, const char* str);

/* local data */
int32 canBPS = 1000000;             // CAN network bit rate
int16 canNodeID = -1;                // CANopen node ID

int main(void)
{
    // The libraries define one global object of type
    // CopleyMotionLibraries named cml.
    //
    // This object has a couple handy member functions
    // including this one which enables the generation of
    // a log file for debugging
    cml.SetDebugLevel(LOG_EVERYTHING);

    // Create an object used to access the low level CAN network.
    // This examples assumes that we're using the Copley PCI CAN card.
#if defined( USE_CAN )
    CopleyCAN hw("CAN0");
    hw.SetBaud(canBPS);
#elif defined( WIN32 )
    WinUdpEcatHardware hw("192.168.0.92");
#else
    LinuxEcatHardware hw("eth0");
#endif

    // Open the network object
#if defined( USE_CAN )
    CanOpen net;
#else
    EtherCAT net;
#endif

    const int axisNum = 3;
    int pointsPerAxis = 5;

    vector<int> axisAPosVec;
    vector<int> axisBPosVec;
    vector<int> axisCPosVec;

    // These are the positions that I want each axis to travel to. 
    for (int i = 0; i < pointsPerAxis; i++)
    {
        axisAPosVec.push_back(i * 10);
        axisBPosVec.push_back(i * 20);
        axisCPosVec.push_back(i * 30);
    }

    // Append the last position at the end of the vector again so that we travel there. 
    // Otherwise, the head and tail pointer will match at the last point and not actually
    // travel to it. 
    axisAPosVec.push_back(axisAPosVec[(int)axisAPosVec.size() - 1]);
    axisBPosVec.push_back(axisBPosVec[(int)axisBPosVec.size() - 1]);
    axisCPosVec.push_back(axisCPosVec[(int)axisCPosVec.size() - 1]);

    pointsPerAxis = (int)axisAPosVec.size();

    const Error* err = net.Open(hw);
    showerr(err, "Opening network");

    // Initialize the amplifiers using default settings
    Amp amp[axisNum];
    AmpSettings amp_settings;

    // ME3/ME4 synchPeriod is 2000 usec (2 ms)
    amp_settings.synchPeriod = 2000;

    // Initializing the first axis of a four axis drive
    printf("Initing axis %d\n", 1);
    err = amp[0].Init(net, canNodeID, amp_settings); // address is -1 for first drive on EtherCAT Network
    showerr(err, "Initing axis a");

    // Initializing the second axis of a four axis drive
    printf("Initing axis %d\n", 2);
    err = amp[1].InitSubAxis(amp[0], 2);
    showerr(err, "Initing axis b");

    // Initializing the third axis of a four axis drive
    printf("Initing axis %d\n", 3);
    err = amp[2].InitSubAxis(amp[0], 3);
    showerr(err, "Initing axis c");

    // Reserve 3000 16-bit words in the drive's trace buffer for the position data.
    // Each position value is 32 bits, so 3000 16-bit words can store 1500 positions. 
    // The ME3 has 3 axes. So that's 1500 / 3 = 500 positions per axis. 
    int16 traceBufferReserveSizeObjIndx = 0x250A;
    uint16 subIndex = 0;
    uint16 numberOf16BitWords = pointsPerAxis * 2 * axisNum;
    err = amp[0].sdo.Dnld16(traceBufferReserveSizeObjIndx, subIndex, numberOf16BitWords);
    showerr(err, "reserving space in the trace buffer");

    int16 traceBufferPointer = 0x250B;
    subIndex = 0;
    uint16 offset = 0;
    err = amp[0].sdo.Dnld16(traceBufferPointer, subIndex, offset);
    showerr(err, "reseting the trace buffer pointer to zero (start of buffer)");

    // send all the position data 
    int16 traceMemoryIndex = 0x250C;
    subIndex = 0;

    for (int i = 0; i < (int)axisAPosVec.size(); i++)
    {
        err = amp[0].sdo.Dnld32(traceMemoryIndex, subIndex, axisAPosVec[i]);
        showerr(err, "sending positions to trace buffer");
    }

    for (int i = 0; i < (int)axisBPosVec.size(); i++)
    {
        err = amp[0].sdo.Dnld32(traceMemoryIndex, subIndex, axisBPosVec[i]);
        showerr(err, "sending positions to trace buffer");
    }

    for (int i = 0; i < (int)axisCPosVec.size(); i++)
    {
        err = amp[0].sdo.Dnld32(traceMemoryIndex, subIndex, axisCPosVec[i]);
        showerr(err, "sending positions to trace buffer");
    }

    // R0 = offset into the buffer for the start of this table in units of 32-bit positions.
    int16 cvmRegisterIndex = 0x2600;
    subIndex = 1; // R0
    int offsetIntoTable = 0;
    err = amp[0].sdo.Dnld32(cvmRegisterIndex, subIndex, offsetIntoTable);
    showerr(err, "setting the offset for table 1");

    // R1 = length of table for axis A
    subIndex = 2; // R1
    int lengthOfTable = pointsPerAxis;
    err = amp[0].sdo.Dnld32(cvmRegisterIndex, subIndex, lengthOfTable);
    showerr(err, "setting the length of table 1 in units of 32-bit positions");

    // R2 = head pointer. 
    subIndex = 3; // R2
    int headPointer = pointsPerAxis - 1;
    err = amp[0].sdo.Dnld32(cvmRegisterIndex, subIndex, headPointer);
    showerr(err, "setting the head pointer for table 1");

    // R3 = tail pointer. 
    subIndex = 4; // R3
    int tailPointer = 0;
    err = amp[0].sdo.Dnld32(cvmRegisterIndex, subIndex, tailPointer);
    showerr(err, "setting the tail pointer for table 1");

    // R4 = offset into the buffer for the start of this table in units of 32-bit positions.
    subIndex = 5; // R4
    offsetIntoTable = pointsPerAxis;
    err = amp[0].sdo.Dnld32(cvmRegisterIndex, subIndex, offsetIntoTable);
    showerr(err, "setting the offset for table 2");

    // R5 = length of table for axis B
    subIndex = 6; // R5
    lengthOfTable = pointsPerAxis;
    err = amp[0].sdo.Dnld32(cvmRegisterIndex, subIndex, lengthOfTable);
    showerr(err, "setting the length of table 2 in units of 32-bit positions");

    // R6 = head pointer. 
    subIndex = 7; // R6
    headPointer = pointsPerAxis - 1;
    err = amp[0].sdo.Dnld32(cvmRegisterIndex, subIndex, headPointer);
    showerr(err, "setting the head pointer for table 2");

    // R7 = tail pointer. 
    subIndex = 8; // R7
    tailPointer = 0;
    err = amp[0].sdo.Dnld32(cvmRegisterIndex, subIndex, tailPointer);
    showerr(err, "setting the tail pointer for table 2");

    // R8 = offset into the buffer for the start of this table in units of 32-bit positions.
    subIndex = 9; // R8
    offsetIntoTable = pointsPerAxis * 2;
    err = amp[0].sdo.Dnld32(cvmRegisterIndex, subIndex, offsetIntoTable);
    showerr(err, "setting the offset for table 3");

    // R9 = length of table for axis C
    subIndex = 10; // R9
    lengthOfTable = pointsPerAxis;
    err = amp[0].sdo.Dnld32(cvmRegisterIndex, subIndex, lengthOfTable);
    showerr(err, "setting the length of table 3 in units of 32-bit positions");

    // R10 = head pointer. 
    subIndex = 11; // R10
    headPointer = pointsPerAxis - 1;
    err = amp[0].sdo.Dnld32(cvmRegisterIndex, subIndex, headPointer);
    showerr(err, "setting the head pointer for table 3");

    // R11 = tail pointer. 
    subIndex = 12; // R11
    tailPointer = 0;
    err = amp[0].sdo.Dnld32(cvmRegisterIndex, subIndex, tailPointer);
    showerr(err, "setting the tail pointer for table 3");

    // configure the three inputs
    int16 digitalInputConfigIndex = 0x2192;
    subIndex = 1;
    uint16 configValue = 0x0038; // Bits 0-7 = 0x38 = trigger moves on rising edge of input.
    err = amp[0].sdo.Dnld16(digitalInputConfigIndex, subIndex, configValue);
    showerr(err, "configuring IN1");

    subIndex = 2;
    configValue = 0x1438; // bits 8-11 = 4 = CVM R4 is starting register (R4, R5, R6, R7 are used). Bits 12-13 = 1 = Axis B. 
    err = amp[0].sdo.Dnld16(digitalInputConfigIndex, subIndex, configValue);
    showerr(err, "configuring IN2");

    subIndex = 3;
    configValue = 0x2838; // bits 8-11 = 8 = CVM R8 is starting register (R8, R9, R10, R11 are used). Bits 12-13 = 2 = Axis C. 
    err = amp[0].sdo.Dnld16(digitalInputConfigIndex, subIndex, configValue);
    showerr(err, "configuring IN3");

    getchar();

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
