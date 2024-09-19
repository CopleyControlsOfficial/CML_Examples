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
Notice how the Wago EtherCAT I/O Bus Coupler divides the process
image into 16-bit chunks. This makes it easy for CML to use a
Pmap16 object on each of these chunks, in order to get/set them.
If a PDO contains objects whose size is not a multiple of 8-bits,
use the Pmap16 object to reference all the objects contained in
that first 16-bit section.

For example, the first 16-bits of RPDO 1 in this example contains
five objects. Use a Pmap16 object to get/set any of those five
objects. Use bit masking/shifting to read/write their values. Initialize
the Pmap16 object with the PDO entry that starts that 16-bit group.
I.E. RPDO1 is 32-bits in size and is divided into two Pmap16 objects.
The first Pmap16 object is initialized with 0xF200.1 and the next is
initialized with 0xF200.5.

For a 16-bit chunk of data that contains an object that is not a
multiple of 8-bits, (like RPDO1 in this example), set
"pdo.SetVerifyFixedPdoMapping(false);" so that CML does not compare the
PDO's currently mapped in the coupler with what is trying to be mapped.
If CML performs this comparison and there are differences found, CML
will attempt to change the PDO mapping inside the coupler. The mapping
in the coupler is fixed, so CML should not attempt to change any PDO
mapping.

In order to enable the module, please change the access of the
NodeWrite method in the EtherCAT class (CML_EtherCAT.h) to public
instead of protected. Copley Controls will keep this method protected
in CML, but users who are using non-Copley products with this library
will need to use it with 3rd party EtherCAT devices. NodeWrite is used
to write to the EtherCAT SubDevice Controller (ESC) registers inside
of the device.

The method "UpdateSyncMngrs" in the WagoIoModule class uses the
NodeWrite method to configure the length of the sync managers in the
Wago coupler. The content of these messages was copied from TwinCAT.
Download the ESI file for the coupler from Wago and use it with
TwinCAT (free Beckhoff software). In TwinCAT, scan the network and
select the coupler. Click on the EtherCAT tab. Click "Advanced
Settings" and then "Init Commands." The list of ESC register commands
used to initialize the coupler are listed. The only commands needed
for use with CML deal with "clear sms," "set sm2 (outputs)," and "set
sm3 (inputs)."

If you are experiencing errors transitioning the node to either
SAFE-OP or OP, check the cml.log file generated by CML to read the
AL status code error returned by the Wago coupler. If the coupler is
returning an AL status error code referring to an invalid sync manager
configuration (0x17, 0x1d, or 0x1e), use TwinCAT to verify the
correctness of your ESC register commands used in the "UpdateSyncMngrs"
method in the WagoIoModule class.

*/

// Comment this out to use EtherCAT
//#define USE_CAN

#include <cstdio>
#include <cstdlib>
#include <iostream>

#include "CML.h"

#define MAX_OBJECTS 25
#define MAX_PDOS 25

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

    Pmap16 pMap16OutArr[MAX_OBJECTS];
    Pmap16 pMap16InArr[MAX_OBJECTS];
    int pMapRpdoCount = 0;
    int pMapTpdoCount = 0;

    WagoIoModule()
    {
        this->isSm2RdOnly = true;
        this->isSm3RdOnly = true;
        this->pdoHeartbeatFixed = true;
    }

    const Error* Init(EtherCAT& net, int nodeId)
    {
        const Error* err = Node::Init(net, nodeId);
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

        pMapRpdoCount = 0;
        pMapTpdoCount = 0;

        tpdoProcessImageSizeBytes = 0;
        rpdoProcessImageSizeBytes = 0;

        rpdoObjIndexVec.clear();
        tpdoObjIndexVec.clear();

        pMapInfoVecInputs.clear();
        pMapInfoVecOutputs.clear();

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

        int sizeInBitsSoFar = 0;
        bool containsSingleBitMapping = false;

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
                
                // contains objects that are not a multiple of 8-bits.
                if (sizeInBits & 0x07) 
                {
                    containsSingleBitMapping = true;
                }

                rpdoBitsTotal += sizeInBits; //(sizeInBits >> 3);

                // bits 8-15
                byte subIndex = (var & 0xFF00) >> 8;

                // bits 16-31
                uint16 index = (var & 0xFFFF0000) >> 16;

                cout << "Object" << i + 1 << ": 0x" << std::hex << (int)index << ". Sub-index: " << (int)subIndex << ". Size in bits: " << std::dec << (int)sizeInBits << endl;

                sizeInBitsSoFar += sizeInBits;
                if (sizeInBitsSoFar == 16)
                {
                    sizeInBitsSoFar = 0;

                    PMap16Info newPdoInfoObj;
                    newPdoInfoObj.index = index;
                    newPdoInfoObj.subIndex = subIndex;
                    newPdoInfoObj.slotNum = rpdoObjIndexVec[t] - 0x1600;

                    if ( containsSingleBitMapping ) 
                    {
                        newPdoInfoObj.containsSingleBitMapping = true;
                    }
                    else 
                    {
                        newPdoInfoObj.containsSingleBitMapping = false;
                    }

                    containsSingleBitMapping = false;

                    pMapInfoVecOutputs.push_back(newPdoInfoObj);
                }
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

                // contains objects that are not a multiple of 8-bits.
                if (sizeInBits & 0x07)
                {
                    containsSingleBitMapping = true;
                }

                tpdoBitsTotal += sizeInBits; //(sizeInBits >> 3);

                // bits 8-15
                byte subIndex = (var & 0xFF00) >> 8;

                // bits 16-31
                uint16 index = (var & 0xFFFF0000) >> 16;

                cout << "Object" << i + 1 << ": 0x" << std::hex << (int)index << ". Sub-index: " << (int)subIndex << ". Size in bits: " << std::dec << (int)sizeInBits << endl;

                sizeInBitsSoFar += sizeInBits;
                if (sizeInBitsSoFar == 16)
                {
                    sizeInBitsSoFar = 0;

                    PMap16Info newPdoInfoObj;
                    newPdoInfoObj.index = index;
                    newPdoInfoObj.subIndex = subIndex;
                    newPdoInfoObj.slotNum = tpdoObjIndexVec[t] - 0x1A00;

                    if (containsSingleBitMapping)
                    {
                        newPdoInfoObj.containsSingleBitMapping = true;
                    }
                    else
                    {
                        newPdoInfoObj.containsSingleBitMapping = false;
                    }

                    containsSingleBitMapping = false;

                    pMapInfoVecInputs.push_back(newPdoInfoObj);
                }
            }
        }

        tpdoProcessImageSizeBytes = ( tpdoBitsTotal >> 3 );

        int y = 0;
        while (y < (int)pMapInfoVecOutputs.size()) 
        {
            int slotNum = pMapInfoVecOutputs[y].slotNum;
            if (pMapInfoVecOutputs[y].containsSingleBitMapping) { rpdoArr[rpdoCounter].SetVerifyFixedPdoMapping(false); }
            
            err = pMap16OutArr[pMapRpdoCount].Init(pMapInfoVecOutputs[y].index, pMapInfoVecOutputs[y].subIndex);
            if (err) { return err; }

            err = rpdoArr[rpdoCounter].AddVar(pMap16OutArr[pMapRpdoCount++]);
            if (err) { return err; }

            // create PMap16 objects, map them to PDO's, and insert them into their PMap16 vectors for later use
            int u = y + 1;
            for( u = y + 1; u < (int)pMapInfoVecOutputs.size(); u++ )
            {
                if (pMapInfoVecOutputs[u].slotNum != pMapInfoVecOutputs[u - 1].slotNum) 
                {
                    break;
                }

                if (pMapInfoVecOutputs[u].containsSingleBitMapping) { rpdoArr[rpdoCounter].SetVerifyFixedPdoMapping(false); }
                                
                err = pMap16OutArr[pMapRpdoCount].Init(pMapInfoVecOutputs[u].index, pMapInfoVecOutputs[u].subIndex);
                if (err) { return err; }

                err = rpdoArr[rpdoCounter].AddVar(pMap16OutArr[pMapRpdoCount++]);
                if (err) { return err; }
            }

            err = this->PdoSet( slotNum, rpdoArr[rpdoCounter++], true );
            if (err) { return err; }
            
            y = u;
        }

        y = 0;
        while (y < (int)pMapInfoVecInputs.size())
        {
            int slotNum = pMapInfoVecInputs[y].slotNum;
            if (pMapInfoVecInputs[y].containsSingleBitMapping) { tpdoArr[tpdoCounter].SetVerifyFixedPdoMapping(false); }

            err = pMap16InArr[pMapTpdoCount].Init(pMapInfoVecInputs[y].index, pMapInfoVecInputs[y].subIndex);
            if (err) { return err; }

            err = tpdoArr[tpdoCounter].AddVar(pMap16InArr[pMapTpdoCount++]);
            if (err) { return err; }

            // create PMap16 objects, map them to PDO's, and insert them into their PMap16 vectors for later use
            int u = y + 1;
            for (u = y + 1; u < (int)pMapInfoVecInputs.size(); u++)
            {
                if (pMapInfoVecInputs[u].slotNum != pMapInfoVecInputs[u - 1].slotNum)
                {
                    break;
                }

                if (pMapInfoVecInputs[u].containsSingleBitMapping) { tpdoArr[tpdoCounter].SetVerifyFixedPdoMapping(false); }

                err = pMap16InArr[pMapTpdoCount].Init(pMapInfoVecInputs[u].index, pMapInfoVecInputs[u].subIndex);
                if (err) { return err; }

                err = tpdoArr[tpdoCounter].AddVar(pMap16InArr[pMapTpdoCount++]);
                if (err) { return err; }
            }

            err = this->PdoSet(slotNum, tpdoArr[tpdoCounter++], true);
            if (err) { return err; }

            y = u;
        }

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
        byte sm2ConfigArr[8] = { 0x00, 0x18, ByteCast(rpdoProcessImageSizeBytes), ByteCast(rpdoProcessImageSizeBytes >> 8), 0x64, 0x00, 0x01, 0x00 };
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

    int tpdoCounter = 0;
    int rpdoCounter = 0;

    TPDO tpdoArr[MAX_PDOS];
    RPDO rpdoArr[MAX_PDOS];

    struct PMap16Info {
        uint16 index;
        byte subIndex;
        int slotNum;
        bool containsSingleBitMapping;
    };

    vector<PMap16Info> pMapInfoVecInputs;
    vector<PMap16Info> pMapInfoVecOutputs;
};

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

    cout << "Reading Outputs" << endl;
    for (int i = 0; i < wagoIoModuleObj.pMapRpdoCount; i++)
    {
        cout << (int)wagoIoModuleObj.pMap16OutArr[i].Read() << endl;
    }

    cout << "Reading Inputs" << endl;
    for (int i = 0; i < wagoIoModuleObj.pMapTpdoCount; i++)
    {
        cout << (int)wagoIoModuleObj.pMap16InArr[i].Read() << endl;
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
