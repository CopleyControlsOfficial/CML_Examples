/** \file

Example of how to use a Wago EtherCAT I/O Bus Coupler with 
the Copley Motion Library.

The following example used a bus coupler that had the following 
fixed PDO mapping: 

Total RPDO count: 4

RPDO1: 0x16ff contains 6 objects.
Object1: 0xf200. Sub-index: 1. Size in bits: 1
Object2: 0xf200. Sub-index: 2. Size in bits: 1
Object3: 0xf200. Sub-index: 3. Size in bits: 1
Object4: 0xf200. Sub-index: 4. Size in bits: 1
Object5: 0x0. Sub-index: 0. Size in bits: 12
Object6: 0xf200. Sub-index: 5. Size in bits: 16

RPDO2: 0x1603 contains 2 objects.
Object1: 0x7030. Sub-index: 1. Size in bits: 16
Object2: 0x7030. Sub-index: 2. Size in bits: 16

RPDO3: 0x1600 contains 2 objects.
Object1: 0x7000. Sub-index: 1. Size in bits: 1
Object2: 0x7000. Sub-index: 2. Size in bits: 1

RPDO4: 0x1701 contains 1 objects.
Object1: 0x0. Sub-index: 0. Size in bits: 14

Total TPDO count: 4

TPDO1: 0x1aff contains 7 objects.
Object1: 0xf100. Sub-index: 1. Size in bits: 1
Object2: 0xf100. Sub-index: 2. Size in bits: 1
Object3: 0xf100. Sub-index: 3. Size in bits: 1
Object4: 0xf100. Sub-index: 4. Size in bits: 1
Object5: 0x0. Sub-index: 0. Size in bits: 11
Object6: 0x10f3. Sub-index: 4. Size in bits: 1
Object7: 0xf100. Sub-index: 5. Size in bits: 16

TPDO2: 0x1a02 contains 4 objects.
Object1: 0x6020. Sub-index: 1. Size in bits: 16
Object2: 0x6020. Sub-index: 2. Size in bits: 16
Object3: 0x6020. Sub-index: 3. Size in bits: 16
Object4: 0x6020. Sub-index: 4. Size in bits: 16

TPDO3: 0x1a01 contains 4 objects.
Object1: 0x6010. Sub-index: 1. Size in bits: 1
Object2: 0x6010. Sub-index: 2. Size in bits: 1
Object3: 0x6010. Sub-index: 3. Size in bits: 1
Object4: 0x6010. Sub-index: 4. Size in bits: 1

TPDO4: 0x1b01 contains 1 objects.
Object1: 0x0. Sub-index: 0. Size in bits: 12

CML uses Pmap objects to get/set the bytes of the process image.
Notice how the Wago EtherCAT I/O Bus Coupler divided the process
image into 16-bit chunks. This makes it easy for CML to use a
Pmap16 object on each of these chunks, in order to get/set them.

For a 16-bit chunk of data that contains a mix of
single-bit objects, (like RPDO1 in this example),
set "pdo.SetVerifyFixedPdoMapping(false);" so that CML does not
attempt to change the fixed PDO mapping inside the coupler.

In order to enable the module, please change the access of the 
NodeWrite method in the EtherCAT class (CML_EtherCAT.h) to public 
instead of protected. Copley Controls will keep this method protected 
in CML, but users who are using non-Copley products with this library 
will need to use it with 3rd party EtherCAT devices. NodeWrite is used
to write to the EtherCAT SubDevice Controller (ESC) registers inside 
of the device. The method "UpdateSyncMngrs" in the WagoIoModule class 
uses the NodeWrite method to configure the length of the sync managers 
in the Wago coupler. 

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

/*

Sync Manager 2 of the Wago EtherCAT I/O Module handles all the RPDO data,
which are the commands to the module.

Sync Manager 3 of the Wago EtherCAT I/O Module handles all the TPDO data,
which is the feedback information from the module.

*/
class WagoIoModule : public Node 
{

public:

    WagoIoModule() 
    {
        this->isSm2RdOnly = true;
        this->isSm3RdOnly = true;
        this->pdoHeartbeatFixed = true;
    }

    const Error* Init(EtherCAT& net, int nodeId) 
    {
        const Error* err = Node::Init( net, nodeId );
        if (err) { return err; }
        return ReadSetupOfReadOnlySyncMgrs();        
    }

    /**
     Read the fixed mappings of the two Sync Managers that deal
     with PDO data: SM2 (RPDO's) and SM3 (TPDO's).
    */
    const Error* ReadSetupOfReadOnlySyncMgrs()
    {
        rpdoCount = 0;
        tpdoCount = 0;
        
        tpdoProcessImageSizeBytes = 0;
        rpdoProcessImageSizeBytes = 0;
        
        rpdoObjIndexVec.clear();
        tpdoObjIndexVec.clear();

        uint16 rpdoBitsTotal = 0;
        uint16 tpdoBitsTotal = 0;

        // get all the objects that are mapped inside of this PDO
        const Error* err = this->sdo.Upld8(0x1c12, 0, rpdoCount);
        if (err) return err;

        cout << "Total RPDO count: " << (int)rpdoCount << endl;

        for (int i = 0; i < rpdoCount; i++)
        {
            uint16 rpdoObj = 0;
            err = this->sdo.Upld16(0x1c12, i + 1, rpdoObj);
            if (err) return err;
            rpdoObjIndexVec.push_back(rpdoObj);
        }

        for (int t = 0; t < (int)rpdoObjIndexVec.size(); t++)
        {
            // get all the objects that are mapped inside of this PDO
            byte objCt;
            err = this->sdo.Upld8(rpdoObjIndexVec[t], 0, objCt);
            if (err) return err;

            cout << "RPDO" << t + 1 << ": 0x" << std::hex << (int)rpdoObjIndexVec[t] << " contains " << std::dec << (int)objCt << " objects." << endl;

            byte i;

            for (i = 0; i < objCt; i++)
            {
                uint32 var = 0;
                err = this->sdo.Upld32(rpdoObjIndexVec[t], i + 1, var);
                if (err) return err;

                // bits 0-7
                byte sizeInBits = var & 0xFF;

                rpdoBitsTotal += sizeInBits; //(sizeInBits >> 3);

                // bits 8-15
                byte subIndex = (var & 0xFF00) >> 8;

                // bits 16-31
                uint16 index = (var & 0xFFFF0000) >> 16;

                cout << "Object" << i + 1 << ": 0x" << std::hex << (int)index << ". Sub-index: " << (int)subIndex << ". Size in bits: " << std::dec << (int)sizeInBits << endl;
            }
        }

        rpdoProcessImageSizeBytes = (rpdoBitsTotal >> 3);

        // get all the objects that are mapped inside of this PDO
        err = this->sdo.Upld8(0x1c13, 0, tpdoCount);
        if (err) return err;

        cout << "Total TPDO count: " << (int)tpdoCount << endl;

        for (int i = 0; i < tpdoCount; i++)
        {
            uint16 tpdoObj = 0;
            err = this->sdo.Upld16(0x1c13, i + 1, tpdoObj);
            if (err) return err;
            tpdoObjIndexVec.push_back(tpdoObj);
        }

        for (int t = 0; t < (int)tpdoObjIndexVec.size(); t++)
        {
            // get all the objects that are mapped inside of this PDO
            byte objCt;
            err = this->sdo.Upld8(tpdoObjIndexVec[t], 0, objCt);
            if (err) return err;

            cout << "TPDO" << t + 1 << ": 0x" << std::hex << (int)tpdoObjIndexVec[t] << " contains " << std::dec << (int)objCt << " objects." << endl;

            byte i;

            int bitOffset = 0;
            for (i = 0; i < objCt; i++)
            {
                uint32 var = 0;
                err = this->sdo.Upld32(tpdoObjIndexVec[t], i + 1, var);
                if (err) return err;

                // bits 0-7
                byte sizeInBits = var & 0xFF;

                tpdoBitsTotal += sizeInBits; //(sizeInBits >> 3);

                // bits 8-15
                byte subIndex = (var & 0xFF00) >> 8;

                // bits 16-31
                uint16 index = (var & 0xFFFF0000) >> 16;

                cout << "Object" << i + 1 << ": 0x" << std::hex << (int)index << ". Sub-index: " << (int)subIndex << ". Size in bits: " << std::dec << (int)sizeInBits << endl;
            }
        }

        tpdoProcessImageSizeBytes = (tpdoBitsTotal >> 3);

        return 0;
    }

    // The following are EtherCAT SubDevice Control (ESC) register commands to the coupler, 
    // which configure its sync manager length. The sync managers (SM2 & SM3) are responsible
    // for the flow of process data (PDO's). Their length must match what is already configured
    // in the Wago coupler's fixed process image. 
    const Error* UpdateSyncMngrs(EtherCAT& net) 
    {
        const Error* err = 0;

        // clear sync managers
        byte clearSyncMngrs[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
        err = net.NodeWrite(this, 0x810, 16, clearSyncMngrs);
        if (err) { return err; }

        // set sm2 (outputs)
        byte sm2ConfigArr[8] = { 0x00, 0x18, ByteCast(rpdoProcessImageSizeBytes), ByteCast(rpdoProcessImageSizeBytes >> 8), 0x64, 0x00, 0x01, 0x00};
        err = net.NodeWrite(this, 0x810, 8, sm2ConfigArr);
        if (err) { return err; }

        // set sm3 (inputs)
        byte sm3ConfigArr[8] = { 0x00, 0x24, ByteCast(tpdoProcessImageSizeBytes), ByteCast(tpdoProcessImageSizeBytes >> 8), 0x00, 0x00, 0x01, 0x00 };
        err = net.NodeWrite(this, 0x818, 8, sm3ConfigArr);
        return err;
    }

    private:
        uint16 tpdoProcessImageSizeBytes = 0;
        uint16 rpdoProcessImageSizeBytes = 0;
        byte tpdoCount = 0;
        byte rpdoCount = 0;
        vector<uint16> rpdoObjIndexVec;
        vector<uint16> tpdoObjIndexVec;
};

/**************************************************
* Read I/O from .cci file.
* Store to amp.
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
    WinUdpEcatHardware hw("192.168.0.40");
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
    WagoIoModule wagoIoModuleObj;
    int16 etherCatNodeID = -1;        // EtherCAT node ID
    err = wagoIoModuleObj.Init(net, etherCatNodeID);
    showerr(err, "Initializing I/O module\n");

    printf("IO Module Initialized.\n");

    err = wagoIoModuleObj.PreOpNode();
    showerr(err, "Preopping node");

    uint16 rpdoBase = 0x1600;
    uint16 tpdoBase = 0x1A00;

    RPDO rpdo1;
    uint16 rpdo1ObjIndex = 0x16FF;
    uint16 rpdoEntryObjectIndex1 = 0xF200;

    Pmap16 objectF200sub1;
    err = objectF200sub1.Init(rpdoEntryObjectIndex1, 1);
    showerr(err, "initing 0xF200 sub-index 1 pmap16 object");
    err = rpdo1.AddVar(objectF200sub1);
    showerr(err, "adding 0xF200 sub-index 1 to RPDO1");

    Pmap16 objectF200sub5;
    err = objectF200sub5.Init(rpdoEntryObjectIndex1, 5);
    showerr(err, "initing 0xF200 sub-index 5 pmap16 object");
    err = rpdo1.AddVar(objectF200sub5);
    showerr(err, "adding 0xF200 sub-index 5 to RPDO1");

    int slotNum = rpdo1ObjIndex - rpdoBase;

    // RPDO1 has a single-bit mapping inside of it so set 
    // this property to false
    rpdo1.SetVerifyFixedPdoMapping(false);

    err = wagoIoModuleObj.PdoSet(slotNum, rpdo1, true);
    showerr(err, "setting RPDO1");

    RPDO rpdo2;

    uint16 rpdo2ObjIndex = 0x1603;
    uint16 rpdoEntryObjectIndex2 = 0x7030;

    Pmap16 object7030sub1;
    err = object7030sub1.Init(rpdoEntryObjectIndex2, 1);
    showerr(err, "initing 0x7030 sub-index 1 pmap16 object");
    err = rpdo2.AddVar(object7030sub1);
    showerr(err, "adding 0x7030 sub-index 1 var to RPDO2");

    Pmap16 object7030sub2;
    err = object7030sub2.Init(rpdoEntryObjectIndex2, 2);
    showerr(err, "initing 0x7030 sub-index 2 pmap16 object");
    err = rpdo2.AddVar(object7030sub2);
    showerr(err, "adding 0x7030 sub-index 2 var to RPDO2");

    slotNum = rpdo2ObjIndex - rpdoBase;

    err = wagoIoModuleObj.PdoSet(slotNum, rpdo2, true);
    showerr(err, "setting RPDO2");

    RPDO rpdo3;

    uint16 rpdo3ObjIndex = 0x1600;
    uint16 rpdoEntryObjectIndex3 = 0x7000;

    Pmap16 object7000sub1;
    err = object7000sub1.Init(rpdoEntryObjectIndex3, 1);
    showerr(err, "initing 0x7000 sub-index 1 pmap16 object");
    err = rpdo3.AddVar(object7000sub1);
    showerr(err, "adding 0x7000 sub-index 1 var to RPDO3");

    slotNum = rpdo3ObjIndex - rpdoBase;

    // RPDO3 has a single-bit mapping inside of it so set 
    // this property to false
    rpdo3.SetVerifyFixedPdoMapping(false);

    err = wagoIoModuleObj.PdoSet(slotNum, rpdo3, true);
    showerr(err, "setting RPDO3");

    TPDO tpdo1;
    uint16 tpdo1ObjIndex = 0x1AFF;
    uint16 tpdoEntryObjectIndex1 = 0xF100;

    Pmap16 objectF100sub1;
    err = objectF100sub1.Init(tpdoEntryObjectIndex1, 1);
    showerr(err, "initing 0xF100 sub-index 1 pmap16 object");
    err = tpdo1.AddVar(objectF100sub1);
    showerr(err, "adding 0xF100 sub-index 1 var to TPDO1");

    Pmap16 objectF100sub5;
    err = objectF100sub5.Init(tpdoEntryObjectIndex1, 5);
    showerr(err, "initing 0xF100 sub-index 5 pmap16 object");
    err = tpdo1.AddVar(objectF100sub5);
    showerr(err, "adding 0xF100 sub-index 5 var to TPDO1");

    slotNum = tpdo1ObjIndex - tpdoBase;

    // TPDO1 has a single-bit mapping inside of it so set 
    // this property to false
    tpdo1.SetVerifyFixedPdoMapping(false);

    err = wagoIoModuleObj.PdoSet(slotNum, tpdo1, true);
    showerr(err, "setting TPDO1");

    TPDO tpdo2;
    uint16 tpdo2ObjIndex = 0x1A02;
    uint16 tpdoEntryObjectIndex2 = 0x6020;

    Pmap16 object6020sub1;
    err = object6020sub1.Init(tpdoEntryObjectIndex2, 1);
    showerr(err, "initing 0x6020 sub-index 1 pmap16 object");
    err = tpdo2.AddVar(object6020sub1);
    showerr(err, "adding 0x6020 sub-index 1 var to TPDO2");

    Pmap16 object6020sub2;
    err = object6020sub2.Init(tpdoEntryObjectIndex2, 2);
    showerr(err, "initing 0x6020 sub-index 2 pmap16 object");
    err = tpdo2.AddVar(object6020sub2);
    showerr(err, "adding 0x6020 sub-index 2 var to TPDO2");

    Pmap16 object6020sub3;
    err = object6020sub3.Init(tpdoEntryObjectIndex2, 3);
    showerr(err, "initing 0x6020 sub-index 3 pmap16 object");
    err = tpdo2.AddVar(object6020sub3);
    showerr(err, "adding 0x6020 sub-index 3 var to TPDO2");

    Pmap16 object6020sub4;
    err = object6020sub4.Init(tpdoEntryObjectIndex2, 4);
    showerr(err, "initing 0x6020 sub-index 4 pmap16 object");
    err = tpdo2.AddVar(object6020sub4);
    showerr(err, "adding 0x6020 sub-index 4 var to TPDO2");

    slotNum = tpdo2ObjIndex - tpdoBase;

    err = wagoIoModuleObj.PdoSet(slotNum, tpdo2, true);
    showerr(err, "setting TPDO2");

    TPDO tpdo3;
    uint16 tpdo3ObjIndex = 0x1A01;
    uint16 tpdoEntryObjectIndex3 = 0x6010;

    Pmap16 object6010sub1;
    err = object6010sub1.Init(tpdoEntryObjectIndex3, 1);
    showerr(err, "initing 0x6010 sub-index 1 pmap16 object");
    err = tpdo3.AddVar(object6010sub1);
    showerr(err, "adding 0x6010 sub-index 1 var to TPDO3");

    slotNum = tpdo3ObjIndex - tpdoBase;

    // TPDO3 has a single-bit mapping inside of it so set 
    // this property to false
    tpdo3.SetVerifyFixedPdoMapping(false);

    err = wagoIoModuleObj.PdoSet(slotNum, tpdo3, true);
    showerr(err, "setting TPDO3");

    // sync managers are now ready to be updated
    err = wagoIoModuleObj.UpdateSyncMngrs(net);
    showerr(err, "updating sync manager length");

    err = net.SafeOpNode(&wagoIoModuleObj);
    showerr(err, "safe-oping node");

    // sleep for 500ms to allow the process data thread to reset the watchdog timer
    CML::Thread::sleep(500);

    // transition the node into operation state
    err = wagoIoModuleObj.StartNode();
    showerr(err, "Starting node");

    // get the first 16-bit chunk of RPDO data
    uint16 controlData = objectF200sub1.Read();
    cout << "Control Data: " << controlData << endl;
    cout << "K-Bus Cycle Overrun Flag Disable: " << (controlData & 0x1) << endl;
    cout << "Input Process Data Hold Request: " << ((controlData & 0x2) >> 1) << endl;
    cout << "Output Process Data Hold Request: " << ((controlData & 0x4) >> 2) << endl;
    cout << "Output Process Data Clear Request: " << ((controlData & 0x8) >> 3) << endl;

    // get the diagnostics control word, which is 16 bits in size
    uint16 diagnosticsControlWord = objectF200sub5.Read();
    cout << "Diagnostics Control Word: " << diagnosticsControlWord << endl;

    // set bits 0-3
    objectF200sub1.Write(0xF);
    controlData = objectF200sub1.Read();
    cout << "Control Data: " << controlData << endl;
    cout << "K-Bus Cycle Overrun Flag Disable: " << (controlData & 0x1) << endl;
    cout << "Input Process Data Hold Request: " << ((controlData & 0x2) >> 1) << endl;
    cout << "Output Process Data Hold Request: " << ((controlData & 0x4) >> 2) << endl;
    cout << "Output Process Data Clear Request: " << ((controlData & 0x8) >> 3) << endl;

    // get the input data for the first module
    for (int i = 0; i < 100; i++) 
    {
        cout << "Input Data for Module 1: " << object6020sub1.Read() << endl;
    }

    return 0;
}

/**************************************************/

static void showerr( const Error *err, const char *str )
{
   if( err )
   {
      printf( "Error %s: %s\n", str, err->toString() );
      exit(1);
   }
}

