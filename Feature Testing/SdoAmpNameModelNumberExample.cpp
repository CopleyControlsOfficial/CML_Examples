/** \file

Simple example of reading the drive name, model number, and firmware version. 

This example also sets the input shaping filter (ASCII ID 0x184) in RAM and then 
reads it back out of the drive. The input shaping filter configuration in CME was
"Zero Vibration and Derivative" with Frequency = 0.5 Hz and Damping Ratio = 0. 
The filter values may be different on your drive, so please follow the recommended 
steps below before applying this filter configuration on your drive.

Determine the correct input shaping filter values to send.
 - Connect to the drive over serial using CME. 
 - Open the Configure Filters Button. Click on the "Input Shaping" tab. Configure 
   the filter.
 - Open "Tools" -> "ASCII Command Line". Send this ASCII string to read the input 
   shaping filter configuration from the drive's RAM: "g r0x184 x".

*/

// Comment this out to use EtherCAT
#define USE_CAN

#include <cstdio>
#include <cstdlib>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include "CML.h"

using std::hex;
using std::cout;
using std::endl;
using std::string;
using std::ostringstream;

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

/* local defines */
#define AMPCT 1

/* local data */
int32 canBPS = 1000000;             // CAN network bit rate
const char* canDevice = "CAN0";     // Identifies the CAN device, if necessary
int16 canNodeID = 1;                // CANopen node ID of amp.

// Function to return a vector of byte swapped characters from an array of bytes
vector<char> GetAsciiCharactersFromByteArray(byte* byteArr, int size) {
    vector<char> charVec;
    for (int i = 1; i < size - 1; i = i + 2) {
        if (byteArr[i + 1] != '\0') {
            charVec.push_back(byteArr[i + 1]);
        }
        else {
            break;
        }

        if (byteArr[i] != '\0') {
            charVec.push_back(byteArr[i]);
        }
        else {
            break;
        }
    }

    return charVec;
}

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
    const Error* err = net.Open(hw);
    showerr(err, "Opening network");

    // Initialize the amplifiers using default settings
    Amp amp;
    AmpSettings ampSettingsObj;
    ampSettingsObj.guardTime = 0;
    ampSettingsObj.enableOnInit = false;

    err = amp.Init(net, canNodeID, ampSettingsObj);
    showerr(err, "Initting amp");

    // 0x2000 is the serial-binary interface. Use this to read byte array types from the drive.
    int16 index = 0x2000;
    int16 subIndex = 0;
    int size = 3;

    // 0x0c is the "read parameter" op-code. 
    // ASCII parameter is 0x92 "Drive Name." Bit 15 is set, meaning read from flash memory, so 0x92 becomes 0x1092. 
    byte getDriveNameCommand[3] = { 0x0c, 0x92, 0x10 };
    err = amp.Download(index, subIndex, size, getDriveNameCommand);
    showerr(err, "requesting to read the drive name in Flash memory.");

    byte driveName[100];
    int sizeNew = 40;
    // please note: sizeNew is passed by reference and is overwritten by the upload method to the number of bytes retrieved.
    err = amp.sdo.Upload(index, subIndex, sizeNew, driveName);
    showerr(err, "reading the serial-binary interface");

    // first byte is error byte indicating whether or not there was a problem reading the drive name
    if (driveName[0] != 0) {
        printf("Error reading drive name!\n");
        return -1;
    }

    // ignore the first byte (error indicator).
    // byte swap the rest of the bytes.
    vector<char> driveNameCharVec = GetAsciiCharactersFromByteArray(driveName, sizeNew);

    printf("Drive Name: ");
    for (int i = 0; i < driveNameCharVec.size(); i++) {
        printf("%c", driveNameCharVec[i]);
    }
    printf("\n");

    // 0x0c is the "read parameter" op-code. 
    // ASCII parameter is 0x80 "Drive Model." Bit 15 is set, meaning read from flash memory, so 0x80 becomes 0x1080. 
    byte getDriveModelCommand[3] = { 0x0c, 0x80, 0x10 };
    err = amp.Download(index, subIndex, size, getDriveModelCommand);
    showerr(err, "requesting to read the drive name in Flash memory.");

    byte driveModel[100];
    err = amp.sdo.Upload(index, subIndex, sizeNew, driveModel);
    showerr(err, "reading the serial-binary interface");

    // first byte is error byte indicating whether or not there was a problem reading the drive model
    if (driveModel[0] != 0) {
        printf("Error reading drive model!\n");
        return -1;
    }

    // ignore the first byte (error indicator).
    // byte swap the rest of the bytes.
    vector<char> driveModelCharVec = GetAsciiCharactersFromByteArray(driveModel, sizeNew);

    printf("Drive Model: ");
    for (int i = 0; i < driveModelCharVec.size(); i++) {
        printf("%c", driveModelCharVec[i]);
    }
    printf("\n");

    index = 0x2384;
    subIndex = 24;
    short firmwareVersion = 0;
    err = amp.sdo.Upld16(index, subIndex, firmwareVersion);
    showerr(err, "reading the firmware version from the drive");

    int firmwareInt = firmwareVersion;

    // use an ostringstream object with a hex manipulator to convert int to hex string.
    ostringstream ss;
    ss << hex << firmwareInt;
    string firmwareVersionStr = ss.str();

    // insert a decimal for clarity
    string decimal = ".";
    firmwareVersionStr.insert(firmwareVersionStr.size() - 2, decimal);
    cout << "Firmware Version: " << firmwareVersionStr << std::endl;

    // How to read/write the input shaping filter (ASCII ID 0x184) over the network. 
    // Step 1: Determine the values to send.
    //  - Connect to the drive over serial using CME. 
    //  - Open the Configure Filters Button. Click on the "Input Shaping" tab. Configure the filter.
    //  - Open "Tools" -> "ASCII Command Line". Send this ASCII string to read the input shaping filter configuration from the drive's RAM: "g r0x184 x".
    // Step 2: Send those values using the code below. 
    // 

    // I used CME to read the value of 0x184 using the ASCII Command Line: "g r0x184 x"
    // The values that came back are: 
    // 0x00000002 0x3f000000 0x00000000 0x00000002 0x00000000 0x3e800000 0x3f800000 0x3f000000 0x40000000 0x3e800000
    byte newInputShapingFilterConfig[43] = 
    {
        0x0d, // 0x0d is the "set parameter" op-code
        0x84, 0x01, // 0x0184 is the ASCII ID for the input shaping filter parameter
        0x00, 0x00, 0x02, 0x00, // 0x00000002
        0x00, 0x3f, 0x00, 0x00, // 0x3f000000
        0x00, 0x00, 0x00, 0x00, // 0x00000000
        0x00, 0x00, 0x02, 0x00, // 0x00000002
        0x00, 0x00, 0x00, 0x00, // 0x00000000
        0x80, 0x3e, 0x00, 0x00, // 0x3e800000
        0x80, 0x3f, 0x00, 0x00, // 0x3f800000
        0x00, 0x3f, 0x00, 0x00, // 0x3f000000
        0x00, 0x40, 0x00, 0x00, // 0x40000000
        0x80, 0x3e, 0x00, 0x00, // 0x3e800000
    };

    index = 0x2000; // serial-binary interface
    subIndex = 0;
    size = 43; // number of bytes to send in this serial-binary message.
    err = amp.Download(index, subIndex, size, newInputShapingFilterConfig);
    showerr(err, "requesting to set the input shaping filter configuration.");

    byte response[1];
    size = 1;
    err = amp.Upload(index, subIndex, size, response);
    showerr(err, "reading the serial-binary interface response to the set command");

    // first byte is error byte indicating whether or not there was a problem performing the last serial-binary command.
    if (response[0] != 0)
    {
        printf("Error setting input shaping filter!\n");
        return -1;
    }

    byte getFilterCommand[3] =
    {
        0x0c, // 0x0c is the "read parameter" op-code. 
        0x84, 0x01 // 0x0184 is the ASCII ID for the input shaping filter parameter
    };

    size = 3;
    err = amp.Download(index, subIndex, size, getFilterCommand);
    showerr(err, "requesting to read the input shaping filter.");

    byte inputShapingFilter[100];
    int filterSize = 100;
    // please note: filterSize is passed by reference and is overwritten by the upload method to the number of bytes retrieved.
    err = amp.sdo.Upload(index, subIndex, filterSize, inputShapingFilter);
    showerr(err, "reading the serial-binary interface");

    // first byte is error byte indicating whether or not there was a problem reading the input shaping filter configuration.
    if (inputShapingFilter[0] != 0)
    {
        printf("Error reading input shaping filter!\n");
        return -1;
    }

    printf("Filter Data: ");
    for (int i = 1; i < filterSize; i++)
    {
        printf("%d ", inputShapingFilter[i]);
    }

    printf("\n");
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
