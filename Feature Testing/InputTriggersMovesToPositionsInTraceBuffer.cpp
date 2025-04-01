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
    int bufferSizePerAxis = 5;
    int pointsPerAxis = 10;

    int axisAOffset = 0;
    int axisBOffset = bufferSizePerAxis;
    int axisCOffset = bufferSizePerAxis * 2;

    int headPointerA = bufferSizePerAxis - 1;
    int headPointerB = bufferSizePerAxis - 1;
    int headPointerC = bufferSizePerAxis - 1;

    // stores the positions to travel to.
    vector<int> axisAPosVec;
    vector<int> axisBPosVec;
    vector<int> axisCPosVec;

    // keeps track of the points we already sent.
    int countA = 0;
    int countB = 0; 
    int countC = 0;

    // These are the positions that I want each axis to travel to. 
    for (int i = 0; i < pointsPerAxis; i++)
    {
        axisAPosVec.push_back(i * 10);
        axisBPosVec.push_back(i * 20);
        axisCPosVec.push_back(i * 30);
    }

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
    uint16 numberOf16BitWords = bufferSizePerAxis * 2 * axisNum;
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

    // Fill up the buffers for each axis.
    int initialFill = 0;
    if (pointsPerAxis >= bufferSizePerAxis) 
    {
        initialFill = bufferSizePerAxis;
    }
    else 
    {
        initialFill = pointsPerAxis;
    }

    for (int i = 0; i < initialFill; i++)
    {
        err = amp[0].sdo.Dnld32(traceMemoryIndex, subIndex, axisAPosVec[countA]);
        showerr(err, "sending positions for axis A to trace buffer");
        countA++;
    }

    for (int i = 0; i < initialFill; i++)
    {
        err = amp[0].sdo.Dnld32(traceMemoryIndex, subIndex, axisBPosVec[countB]);
        showerr(err, "sending positions for axis B to trace buffer");
        countB++;
    }

    for (int i = 0; i < initialFill; i++)
    {
        err = amp[0].sdo.Dnld32(traceMemoryIndex, subIndex, axisCPosVec[countC]);
        showerr(err, "sending positions for axis C to trace buffer");
        countC++;
    }

    // R0 = offset into the buffer for the start of this table in units of 32-bit positions.
    int16 cvmRegisterIndex = 0x2600;
    subIndex = 1; // R0
    err = amp[0].sdo.Dnld32(cvmRegisterIndex, subIndex, axisAOffset);
    showerr(err, "setting the offset for table 1");

    // R1 = length of table for axis A
    subIndex = 2; // R1
    int lengthOfTable = bufferSizePerAxis;
    err = amp[0].sdo.Dnld32(cvmRegisterIndex, subIndex, lengthOfTable);
    showerr(err, "setting the length of table 1 in units of 32-bit positions");

    // R2 = head pointer. 
    subIndex = 3; // R2
    err = amp[0].sdo.Dnld32(cvmRegisterIndex, subIndex, headPointerA);
    showerr(err, "setting the head pointer for table 1");

    // R3 = tail pointer. 
    subIndex = 4; // R3
    int tailPointer = 0;
    err = amp[0].sdo.Dnld32(cvmRegisterIndex, subIndex, tailPointer);
    showerr(err, "setting the tail pointer for table 1");

    // R4 = offset into the buffer for the start of this table in units of 32-bit positions.
    subIndex = 5; // R4
    err = amp[0].sdo.Dnld32(cvmRegisterIndex, subIndex, axisBOffset);
    showerr(err, "setting the offset for table 2");

    // R5 = length of table for axis B
    subIndex = 6; // R5
    lengthOfTable = bufferSizePerAxis;
    err = amp[0].sdo.Dnld32(cvmRegisterIndex, subIndex, lengthOfTable);
    showerr(err, "setting the length of table 2 in units of 32-bit positions");

    // R6 = head pointer. 
    subIndex = 7; // R6
    err = amp[0].sdo.Dnld32(cvmRegisterIndex, subIndex, headPointerB);
    showerr(err, "setting the head pointer for table 2");

    // R7 = tail pointer. 
    subIndex = 8; // R7
    tailPointer = 0;
    err = amp[0].sdo.Dnld32(cvmRegisterIndex, subIndex, tailPointer);
    showerr(err, "setting the tail pointer for table 2");

    // R8 = offset into the buffer for the start of this table in units of 32-bit positions.
    subIndex = 9; // R8
    err = amp[0].sdo.Dnld32(cvmRegisterIndex, subIndex, axisCOffset);
    showerr(err, "setting the offset for table 3");

    // R9 = length of table for axis C
    subIndex = 10; // R9
    lengthOfTable = bufferSizePerAxis;
    err = amp[0].sdo.Dnld32(cvmRegisterIndex, subIndex, lengthOfTable);
    showerr(err, "setting the length of table 3 in units of 32-bit positions");

    // R10 = head pointer. 
    subIndex = 11; // R10
    err = amp[0].sdo.Dnld32(cvmRegisterIndex, subIndex, headPointerC);
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

    bool axisADone = false;
    if (countA == (int)axisAPosVec.size()) 
    {
        axisADone = true;
    }

    bool axisBDone = false;
    if (countB == (int)axisBPosVec.size())
    {
        axisBDone = true;
    }

    bool axisCDone = false;
    if (countC == (int)axisCPosVec.size())
    {
        axisCDone = true;
    }

    // R3 = tail pointer. 
    subIndex = 4; // R3
    int tailPointer1_Old = 0;
    err = amp[0].sdo.Upld32(cvmRegisterIndex, subIndex, tailPointer1_Old);
    showerr(err, "reading the tail pointer for table 1");

    // R7 = tail pointer. 
    subIndex = 8; // R7
    int tailPointer2_Old = 0;
    err = amp[0].sdo.Upld32(cvmRegisterIndex, subIndex, tailPointer2_Old);
    showerr(err, "reading the tail pointer for table 2");

    // R11 = tail pointer. 
    subIndex = 12; // R11
    int tailPointer3_Old = 0;
    err = amp[0].sdo.Upld32(cvmRegisterIndex, subIndex, tailPointer3_Old);
    showerr(err, "reading the tail pointer for table 3");

    bool lastPointA = false;
    bool lastPointB = false;
    bool lastPointC = false;

    // send the rest of the points if there are any left to send
    while ((axisADone == false) || (axisBDone == false) || (axisCDone == false)) 
    {
        // R3 = tail pointer. 
        subIndex = 4; // R3
        int tailPointer1_New = 0;
        err = amp[0].sdo.Upld32(cvmRegisterIndex, subIndex, tailPointer1_New);
        showerr(err, "reading the tail pointer for table 1");

        // R7 = tail pointer. 
        subIndex = 8; // R7
        int tailPointer2_New = 0;
        err = amp[0].sdo.Upld32(cvmRegisterIndex, subIndex, tailPointer2_New);
        showerr(err, "reading the tail pointer for table 2");

        // R11 = tail pointer. 
        subIndex = 12; // R11
        int tailPointer3_New = 0;
        err = amp[0].sdo.Upld32(cvmRegisterIndex, subIndex, tailPointer3_New);
        showerr(err, "reading the tail pointer for table 3");

        // firmware incremented the tail pointer. Therefore, there is a free slot available
        // in the trace buffer. Let's go fill it!
        if ((tailPointer1_New != tailPointer1_Old) && (axisADone == false)) 
        {
            // if we are not sending the last point, use this loop
            if (lastPointA == false) 
            {
                // use a while loop because there could be more than one open slot available
                while ((tailPointer1_Old != tailPointer1_New) && (lastPointA == false))
                {
                    // travel to the open slot
                    tailPointer1_Old++;
                    tailPointer1_Old %= bufferSizePerAxis;

                    headPointerA++;
                    headPointerA %= bufferSizePerAxis;

                    // R2 = head pointer. 
                    subIndex = 3; // R2
                    err = amp[0].sdo.Dnld32(cvmRegisterIndex, subIndex, headPointerA);
                    showerr(err, "setting the head pointer for table 1");

                    // fill the open slot

                    // set the pointer (address) into the trace buffer
                    subIndex = 0;
                    offset = (axisAOffset * 2) + (headPointerA * 2);
                    err = amp[0].sdo.Dnld16(traceBufferPointer, subIndex, offset);
                    showerr(err, "setting the trace buffer pointer for axis A");

                    // send the position (value) in the trace buffer
                    err = amp[0].sdo.Dnld32(traceMemoryIndex, subIndex, axisAPosVec[countA]);
                    showerr(err, "sending positions for axis A to trace buffer");
                    countA++;

                    if (countA == (int)axisAPosVec.size()) 
                    { 
                        countA--;
                        lastPointA = true;
                    }
                }
            }
            // we are sending the last point (twice because we want to travel to it).
            else 
            {
                headPointerA++;
                headPointerA %= bufferSizePerAxis;

                // R2 = head pointer. 
                subIndex = 3; // R2
                err = amp[0].sdo.Dnld32(cvmRegisterIndex, subIndex, headPointerA);
                showerr(err, "setting the head pointer for table 1");

                // fill the open slot

                // set the pointer (address) into the trace buffer
                subIndex = 0;
                offset = (axisAOffset * 2) + (headPointerA * 2);
                err = amp[0].sdo.Dnld16(traceBufferPointer, subIndex, offset);
                showerr(err, "setting the trace buffer pointer for axis A");

                // send the position (value) in the trace buffer
                err = amp[0].sdo.Dnld32(traceMemoryIndex, subIndex, axisAPosVec[countA]);
                showerr(err, "sending positions for axis A to trace buffer");

                axisADone = true;
            }
        }

        // firmware incremented the tail pointer. Therefore, there is a free slot available
        // in the trace buffer. Let's go fill it!
        if ((tailPointer2_New != tailPointer2_Old) && (axisBDone == false))
        {
            // if we are not sending the last point, use this loop
            if (lastPointB == false)
            {
                // use a while loop because there could be more than one open slot available
                while ((tailPointer2_Old != tailPointer2_New) && (lastPointB == false))
                {
                    // travel to the open slot
                    tailPointer2_Old++;
                    tailPointer2_Old %= bufferSizePerAxis;

                    headPointerB++;
                    headPointerB %= bufferSizePerAxis;

                    // R6 = head pointer. 
                    subIndex = 7; // R6
                    err = amp[0].sdo.Dnld32(cvmRegisterIndex, subIndex, headPointerB);
                    showerr(err, "setting the head pointer for table 2");

                    // fill the open slot

                    // set the pointer (address) into the trace buffer
                    subIndex = 0;
                    offset = (axisBOffset * 2) + (headPointerB * 2);
                    err = amp[0].sdo.Dnld16(traceBufferPointer, subIndex, offset);
                    showerr(err, "setting the trace buffer pointer for axis B");

                    // send the position (value) in the trace buffer
                    err = amp[0].sdo.Dnld32(traceMemoryIndex, subIndex, axisBPosVec[countB]);
                    showerr(err, "sending positions for axis B to trace buffer");
                    countB++;

                    if (countB == (int)axisBPosVec.size())
                    {
                        countB--;
                        lastPointB = true;
                    }
                }
            }
            // we are sending the last point (twice because we want to travel to it).
            else
            {
                headPointerB++;
                headPointerB %= bufferSizePerAxis;

                // R6 = head pointer. 
                subIndex = 7; // R6
                err = amp[0].sdo.Dnld32(cvmRegisterIndex, subIndex, headPointerB);
                showerr(err, "setting the head pointer for table 2");

                // fill the open slot

                // set the pointer (address) into the trace buffer
                subIndex = 0;
                offset = (axisBOffset * 2) + (headPointerB * 2);
                err = amp[0].sdo.Dnld16(traceBufferPointer, subIndex, offset);
                showerr(err, "setting the trace buffer pointer for axis B");

                // send the position (value) in the trace buffer
                err = amp[0].sdo.Dnld32(traceMemoryIndex, subIndex, axisBPosVec[countB]);
                showerr(err, "sending positions for axis B to trace buffer");

                axisBDone = true;
            }
        }

        // firmware incremented the tail pointer. Therefore, there is a free slot available
        // in the trace buffer. Let's go fill it!
        if ((tailPointer3_New != tailPointer3_Old) && (axisCDone == false))
        {
            // if we are not sending the last point, use this loop
            if (lastPointC == false)
            {
                // use a while loop because there could be more than one open slot available
                while ((tailPointer3_Old != tailPointer3_New) && (lastPointC == false))
                {
                    // travel to the open slot
                    tailPointer3_Old++;
                    tailPointer3_Old %= bufferSizePerAxis;

                    headPointerC++;
                    headPointerC %= bufferSizePerAxis;

                    // R10 = head pointer. 
                    subIndex = 11; // R10
                    err = amp[0].sdo.Dnld32(cvmRegisterIndex, subIndex, headPointerC);
                    showerr(err, "setting the head pointer for table 3");

                    // fill the open slot

                    // set the pointer (address) into the trace buffer
                    subIndex = 0;
                    offset = (axisCOffset * 2) + (headPointerC * 2);
                    err = amp[0].sdo.Dnld16(traceBufferPointer, subIndex, offset);
                    showerr(err, "setting the trace buffer pointer for axis C");

                    // send the position (value) in the trace buffer
                    err = amp[0].sdo.Dnld32(traceMemoryIndex, subIndex, axisCPosVec[countC]);
                    showerr(err, "sending positions for axis C to trace buffer");
                    countC++;

                    if (countC == (int)axisCPosVec.size())
                    {
                        countC--;
                        lastPointC = true;
                    }
                }
            }
            // we are sending the last point (twice because we want to travel to it).
            else
            {
                headPointerC++;
                headPointerC %= bufferSizePerAxis;

                // R10 = head pointer. 
                subIndex = 11; // R10
                err = amp[0].sdo.Dnld32(cvmRegisterIndex, subIndex, headPointerC);
                showerr(err, "setting the head pointer for table 3");

                // fill the open slot

                // set the pointer (address) into the trace buffer
                subIndex = 0;
                offset = (axisCOffset * 2) + (headPointerC * 2);
                err = amp[0].sdo.Dnld16(traceBufferPointer, subIndex, offset);
                showerr(err, "setting the trace buffer pointer for axis C");

                // send the position (value) in the trace buffer
                err = amp[0].sdo.Dnld32(traceMemoryIndex, subIndex, axisCPosVec[countC]);
                showerr(err, "sending positions for axis C to trace buffer");

                axisCDone = true;
            }
        }
    }

    printf("\nDone! All data loaded into trace buffer.\n");
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
