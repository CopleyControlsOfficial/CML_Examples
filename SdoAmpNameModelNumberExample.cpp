/** \file

Simple example of reading the drive name, model number, and firmware version.

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
const char* canDevice = "CAN0";           // Identifies the CAN device, if necessary
int16 canNodeID = 1;                // CANopen node ID of first amp.  Second will be ID+1, etc.

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

/**************************************************
* Just home the motor and do a bunch of random
* moves.
**************************************************/
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

    // Set the Trajectory Profile Mode (ASCII Parameter 0xc8) to 0x0100 (bit 8 set) = Relative Move
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

