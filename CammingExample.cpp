/*

CammingExample.cpp

Below is an example of how to load three CAM tables in a drive's
RAM and configure the drive to use these CAM tables. 

- First, space is allocated in the drive's internal trace buffer.

- Second, bit 7 of the camming configuration object (0x2360) is 
set so that the drive will use the camming tables in RAM. 

- Third, the drive's desired state is set to a value of 25, which
is Camming Mode. 

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
    WinUdpEcatHardware hw("192.168.0.205");
#else
    LinuxEcatHardware hw("eth0");
#endif

    // Open the network object
#if defined( USE_CAN )
    CanOpen net;
#else
    EtherCAT net;
#endif

    const Error* err = net.Open(hw);
    showerr(err, "Opening network");

    // Initialize the amplifier using default settings
    Amp amp;
    printf("Doing init\n");
    err = amp.Init(net, canNodeID);
    showerr(err, "Initting amp");

    // reserve 36 16-bit words in the drive's trace buffer for the camming tables.
    int16 traceBufferReserveSizeObjIndx = 0x250A;
    uint16 subIndex = 0;
    uint16 numberOf16BitWords = 36;
    err = amp.sdo.Dnld16(traceBufferReserveSizeObjIndx, subIndex, numberOf16BitWords);
    showerr(err, "reserving space in the trace buffer");

    // 0x2000 is the serial-binary interface. Use this to read/write byte array types from/to the drive.
    int16 index = 0x2000;
    int size = 75; // number of bytes = (36 16-bit words of trace buffer data * 2) + 3 byte header = 75 bytes total

    //// 0x0f is the "trace command" op-code. 
    //// 0x0016 is the function code for downloading data in the trace buffer
    //// 0x0006 is the starting address for CAM TABLE 0
    //// 0x000a is the size of CAM TABLE 0 in number of 16-bit words
    //// 0x0010 is the starting address for CAM TABLE 1
    //// 0x000a is the size of CAM TABLE 1 in number of 16-bit words
    //// 0x001a is the starting address for CAM TABLE 2
    //// 0x000a is the size of CAM TABLE 2 in number of 16-bit words
    //// CAM TABLE 0
    //// (0x0064, 0x0001)
    //// (0x0063, 0x0002)
    //// (0x0062, 0x0003)
    //// (0x0061, 0x0004)
    //// (0x0060, 0x0005)
    //// CAM TABLE 1
    //// (0x005f, 0x0006)
    //// (0x005e, 0x0007)
    //// (0x005d, 0x0008)
    //// (0x005c, 0x0009)
    //// (0x005b, 0x000a)
    //// CAM TABLE 2
    //// (0x005a, 0x000b)
    //// (0x0059, 0x000c)
    //// (0x0058, 0x000d)
    //// (0x0057, 0x000e)
    //// (0x0056, 0x000f)
    byte traceBufferData[75] = { 
        0x0f, 
        0x16, 0x00, 
        0x06, 0x00, 
        0x0a, 0x00, 
        0x10, 0x00, 
        0x0a, 0x00, 
        0x1a, 0x00, 
        0x0a, 0x00, 
        0x64, 0x00, 0x01, 0x00,
        0x63, 0x00, 0x02, 0x00,
        0x62, 0x00, 0x03, 0x00,
        0x61, 0x00, 0x04, 0x00, 
        0x60, 0x00, 0x05, 0x00, 
        0x5f, 0x00, 0x06, 0x00,
        0x5e, 0x00, 0x07, 0x00,
        0x5d, 0x00, 0x08, 0x00,
        0x5c, 0x00, 0x09, 0x00,
        0x5b, 0x00, 0x0a, 0x00,
        0x5a, 0x00, 0x0b, 0x00,
        0x59, 0x00, 0x0c, 0x00,
        0x58, 0x00, 0x0d, 0x00,
        0x57, 0x00, 0x0e, 0x00,
        0x56, 0x00, 0x0f, 0x00,
    };
    err = amp.Download(index, subIndex, size, traceBufferData);
    showerr(err, "requesting to read the drive name in Flash memory.");

    byte responseFromDrive[1];
    int sizeNew = 1;
    // please note: sizeNew is passed by reference and is overwritten by the upload method to the number of bytes retrieved.
    err = amp.sdo.Upload(index, subIndex, sizeNew, responseFromDrive);
    showerr(err, "reading the response to the cam table download");

    // first byte is error byte indicating whether or not there was a problem reading the drive name
    if (responseFromDrive[0] != 0) 
    {
        printf("Error loading camming tables in RAM!\n");
        return -1;
    }

    // set bit 7 of the Camming Configuration object (0x2360) to configure the drive to run CAM tables from RAM. 
    int16 cammingConfigObjIndx = 0x2360;
    uint16 cammingConfigValue = 0;
    err = amp.sdo.Upld16(cammingConfigObjIndx, subIndex, cammingConfigValue);
    showerr(err, "reading the camming configuration object");
    
    // set bit 7 by OR-ing the value with 0x80
    cammingConfigValue |= 0x80;
    err = amp.sdo.Dnld16(cammingConfigObjIndx, subIndex, cammingConfigValue);
    showerr(err, "setting the camming configuration object");

    int16 desiredState = 0x2300;
    uint16 cammingMode = 25;
    err = amp.sdo.Dnld16(desiredState, subIndex, cammingMode);
    showerr(err, "setting desired state to camming mode");

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
