/*

BeckhoffAsicChip.cpp

Example interface for the Beckhoff ASIC Chip.

The ASIC chip does not support CoE (does not have SDO support because it doesn't have a mailbox).
Therefore, the NodeRead and NodeWrite methods are used to send and receive data between CML and
the ASIC chip.

EtherCAT::NodeRead sends an FPRD datagram over EtherCAT to write to a node given its Node ID.
EtherCAT::NodeWrite sends an FPWR datagram over EtherCAT to write to a node given its Node ID.

To use these two methods, simply move them from "private" to "public" in the EtherCAT class. 

*/

#include <cstdio>
#include <cstdlib>
#include <iostream>

#include "CML.h"

using std::cout;
using std::endl;

#if defined( WIN32 )
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
int16 etherCatNodeID = 1002;        // EtherCAT node ID

class BeckhoffAsicChip : public Node{};

int main(void)
{
    // The libraries define one global object of type
    // CopleyMotionLibraries named cml.
    //
    // This object has a couple handy member functions
    // including this one which enables the generation of
    // a log file for debugging
    cml.SetDebugLevel(LOG_EVERYTHING);

#if defined( WIN32 )
    WinUdpEcatHardware hw("192.168.0.205");
#else
    LinuxEcatHardware hw("eth0");
#endif

    // Open the network object
    EtherCAT net;

    const Error* err = net.Open(hw);
    showerr(err, "Opening network");

    BeckhoffAsicChip beckhoffIoModule;
    err = beckhoffIoModule.Init(net, etherCatNodeID);
    showerr(err, "initializing Beckhoff IO Module");

    cout << "Node Alias: " << beckhoffIoModule.GetNodeID() << endl;

    printf("Beckhoff ASIC I/O Module successfully initialized.\n");

    err = beckhoffIoModule.PreOpNode();
    showerr(err, "Preopping node");

    err = beckhoffIoModule.StartNode();
    showerr(err, "Starting node");

    int32 outputVal = 0;

    int count = 0;

    // individually set each output one at a time in an endless loop
    while (1)
    {
        err = net.NodeWrite(&beckhoffIoModule, 0x0f02, 2, outputVal);
        showerr(err, "setting outputs");

        uint8 inputPinStates[2];
        err = net.NodeRead(&beckhoffIoModule, 0x1000, 2, inputPinStates);

        uint8 outputPinStates[2];
        err = net.NodeRead(&beckhoffIoModule, 0x0f02, 2, outputPinStates);

        int inputPinStates_1_to_8 = inputPinStates[0];
        int inputPinStates_9_to_16 = inputPinStates[1];
        int outputPinStates_1_to_8 = outputPinStates[0];
        int outputPinStates_9_to_16 = outputPinStates[1];

        cout << "Input pin states 1 to 8 are: " << inputPinStates_1_to_8 << endl;
        cout << "Input pin states 9 to 16 are: " << inputPinStates_9_to_16 << endl;
        cout << "Output pin states 1 to 8 are: " << outputPinStates_1_to_8 << endl;
        cout << "Output pin states 9 to 16 are: " << outputPinStates_9_to_16 << endl;

        outputVal = outputVal | (1 << count);
        count++;

        if (outputVal > 0xFFFF)
        {
            count = 0;
            outputVal = 0;
        }

        CML::Thread::sleep(1000);
    }

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

