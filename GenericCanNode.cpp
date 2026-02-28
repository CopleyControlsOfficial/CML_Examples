/*

GenericCanNode.cpp

The following is an example of how to command motion using a non-Copley drive
on a CAN bus. In this example, CAN node ID 1 is not a Copley drive. CAN node
ID 2 is a Copley drive and is generating the SYNC pulse every 100ms. The PDO 
mapping of the non-Copley drive is below.

RPDO1: Control Word (0x6040)
RPDO2: Control Word (0x6040), Mode of Operation (0x6060)
RPDO3: Control Word (0x6040), Target Position (0x607a)
RPDO4: Control Word (0x6040), Target Velocity (0x60ff)

TPDO1: Status Word (0x6041)
TPDO2: Status Word (0x6041), Mode of Operation Display (0x6061)
TPDO3: Status Word (0x6041), Actual Motor Position (0x6064)
TPDO4: Status Word (0x6041), Actual Motor Velocity (0x606c)

*/

#include <CML.h>
#include <can/can_copley.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>

CML_NAMESPACE_USE();

static void showerr(const Error* err, const char* msg);

class RPDO1 : public RPDO
{
    uint32 netRef;
    Pmap16 controlWord;

public:

    RPDO1() {}
    const Error* Init(Node& nodeIn, uint16 slotIn, int canMessageIdIn)
    {
        netRef = nodeIn.GetNetworkRef();

        // Init the base class
        const Error* err = RPDO::Init( canMessageIdIn );

        // Init the mapping objects that describe the data mapped to this PDO
        if (!err) err = controlWord.Init( OBJID_CONTROL );

        // Add these variables to the PDO
        if (!err) err = AddVar( controlWord );

        // Set the PDO type so that it's data will be acted on immediately
        if (!err) err = SetType( 255 );

        // Program this PDO
        if (!err) err = nodeIn.PdoSet( slotIn, *this );

        return err;
    }

    const Error* Transmit( uint16 controlWordValueIn )
    {
        controlWord.Write( controlWordValueIn );
        RefObjLocker<Network> net( netRef );
        if( !net ) return &NodeError::NetworkUnavailable;
        return RPDO::Transmit( *net );
    }
};

class RPDO2 : public RPDO
{
    uint32 netRef;
    Pmap16 controlWord;
    Pmap8 modeOfOperation;

public:

    RPDO2() {}
    const Error* Init( Node& nodeIn, uint16 slotIn, int canMessageIdIn )
    {
        netRef = nodeIn.GetNetworkRef();

        // Init the base class
        const Error* err = RPDO::Init( canMessageIdIn );
        if( !err ) err = controlWord.Init( OBJID_CONTROL );
        if( !err ) err = modeOfOperation.Init( OBJID_OP_MODE );
        if( !err ) err = AddVar( controlWord );
        if( !err ) err = AddVar( modeOfOperation );
        if (!err) err = SetType( 255 );
        if (!err) err = nodeIn.PdoSet( slotIn, *this );

        return err;
    }

    const Error* Transmit( uint16 controlWordValueIn, uint8 modeOfOpIn )
    {
        controlWord.Write( controlWordValueIn );
        modeOfOperation.Write( modeOfOpIn );
        RefObjLocker<Network> net( netRef );
        if( !net ) return &NodeError::NetworkUnavailable;
        return RPDO::Transmit( *net );
    }
};

class RPDO3 : public RPDO
{
    uint32 netRef;
    Pmap16 controlWord;
    Pmap32 targetPosition;

public:

    RPDO3() {}
    const Error* Init( Node& nodeIn, uint16 slotIn, int canMessageIdIn )
    {
        netRef = nodeIn.GetNetworkRef();

        // Init the base class
        const Error* err = RPDO::Init( canMessageIdIn );
        if( !err ) err = controlWord.Init( OBJID_CONTROL );
        if( !err ) err = targetPosition.Init( OBJID_PROFILE_POS );
        if( !err ) err = AddVar( controlWord );
        if( !err ) err = AddVar( targetPosition );
        if( !err ) err = SetType( 255 );
        if( !err ) err = nodeIn.PdoSet( slotIn, *this );
        return err;
    }

    const Error* Transmit( uint16 controlWordValueIn, int32 targetPositionIn )
    {
        controlWord.Write( controlWordValueIn );
        targetPosition.Write( targetPositionIn );
        RefObjLocker<Network> net( netRef );
        if( !net ) return &NodeError::NetworkUnavailable;
        return RPDO::Transmit( *net );
    }
};

class RPDO4 : public RPDO
{
    uint32 netRef;
    Pmap16 controlWord;
    Pmap32 targetVelocity;

public:

    RPDO4() {}
    const Error* Init( Node& nodeIn, uint16 slotIn, int canMessageIdIn )
    {
        netRef = nodeIn.GetNetworkRef();
        const Error* err = RPDO::Init( canMessageIdIn );
        if( !err ) err = controlWord.Init( OBJID_CONTROL );
        if( !err ) err = targetVelocity.Init( OBJID_TARGET_VEL );
        if( !err ) err = AddVar( controlWord );
        if( !err ) err = AddVar( targetVelocity );
        if( !err ) err = SetType( 255 );
        if( !err ) err = nodeIn.PdoSet( slotIn, *this );
        return err;
    }

    const Error* Transmit( uint16 controlWordValueIn, int32 targetVelocityIn )
    {
        controlWord.Write( controlWordValueIn );
        targetVelocity.Write( targetVelocityIn );
        RefObjLocker<Network> net( netRef );
        if( !net ) return &NodeError::NetworkUnavailable;
        return RPDO::Transmit( *net );
    }
};

class TPDO1 : public TPDO
{
public:
    Pmap16 statusWord;

    // Default constructor does nothing
    TPDO1() {}

    // Called once at startup to map the PDO and configure CML to 
    // listen for it.
    const Error* Init( Node& node, int slot, int canMessageId )
    {
        // Initialize the transmit PDO base class.
        // This needs to know the CANopen network and CAN message ID 
        // associated with the PDO.
        const Error* err = TPDO::Init( canMessageId );

        // Set bit 30 of the CAN message ID to disable RTR for this TPDO
        SetRtrOk( 0 );
        if( !err ) err = SetType( 255 );
        if( !err ) err = statusWord.Init( OBJID_STATUS, 0 );
        if( !err ) err = AddVar( statusWord );
        if( !err ) err = node.PdoSet( slot, *this );
        return err;
    }

    // This function is called when the PDO is received
    virtual void Received(void)
    {
        // nothing to do at this time
    }
};

class TPDO2 : public TPDO
{
public:
    Pmap16 statusWord;
    Pmap8 modeOfOpDisplay;

    TPDO2() {}
    const Error* Init( Node& node, int slot, int canMessageId )
    {
        const Error* err = TPDO::Init( canMessageId );
        SetRtrOk( 0 ); // Set bit 30 of the CAN message ID to disable RTR for this TPDO
        if( !err ) err = SetType( 255 );
        if( !err ) err = statusWord.Init( OBJID_STATUS, 0 );
        if( !err ) err = modeOfOpDisplay.Init( OBJID_OP_MODE_DISP, 0 );
        if( !err ) err = AddVar( statusWord );
        if( !err ) err = AddVar( modeOfOpDisplay );
        if( !err ) err = node.PdoSet( slot, *this );
        return err;
    }

    virtual void Received(void)
    {
        // nothing to do at this time
    }
};

class TPDO3 : public TPDO
{
public:
    Pmap16 statusWord;
    Pmap32 actualPosition;

    TPDO3() {}
    const Error* Init( Node& node, int slot, int canMessageId )
    {
        const Error* err = TPDO::Init( canMessageId );
        SetRtrOk( 0 ); // Set bit 30 of the CAN message ID to disable RTR for this TPDO
        if( !err ) err = SetType( 1 ); // transmission type = 1 = transmit every SYNC pulse
        if( !err ) err = statusWord.Init( OBJID_STATUS, 0 );
        if( !err ) err = actualPosition.Init( OBJID_POS_ACT, 0 );
        if( !err ) err = AddVar( statusWord );
        if( !err ) err = AddVar( actualPosition );
        if( !err ) err = node.PdoSet( slot, *this );
        return err;
    }

    virtual void Received(void)
    {
        // nothing to do at this time
    }
};

class TPDO4 : public TPDO
{
public:
    Pmap16 statusWord;
    Pmap32 actualVelocity;

    TPDO4() {}
    const Error* Init( Node& node, int slot, int canMessageId )
    {
        const Error* err = TPDO::Init( canMessageId );
        SetRtrOk( 0 ); // Set bit 30 of the CAN message ID to disable RTR for this TPDO
        if( !err ) err = SetType( 1 ); // transmission type = 1 = transmit every SYNC pulse
        if( !err ) err = statusWord.Init( OBJID_STATUS, 0 );
        if( !err ) err = actualVelocity.Init( 0x606C, 0 );
        if( !err ) err = AddVar( statusWord );
        if( !err ) err = AddVar( actualVelocity );
        if( !err ) err = node.PdoSet( slot, *this );
        return err;
    }

    virtual void Received(void)
    {
        // nothing to do at this time
    }
};

void InitNode( Node& drive, Network& net, int canNodeIdIn, RPDO1& rpdo1In, RPDO2& rpdo2In, RPDO3& rpdo3In, RPDO4& rpdo4In, TPDO1& tpdo1In, TPDO2& tpdo2In, TPDO3& tpdo3In, TPDO4& tpdo4In )
{
    printf( "Initting drive %d\n", canNodeIdIn );
    const Error* err = drive.Init( net, canNodeIdIn );
    showerr( err, "Initting amp" );

    err = drive.PreOpNode();
    showerr( err, "pre-oping node" );

    err = rpdo1In.Init( drive, 0, 0x200 + canNodeIdIn );
    showerr( err, "initializing RPDO1" );

    err = rpdo2In.Init( drive, 1, 0x300 + canNodeIdIn );
    showerr( err, "initializing RPDO2" );

    err = rpdo3In.Init( drive, 2, 0x400 + canNodeIdIn );
    showerr(err, "initializing RPDO3");

    err = rpdo4In.Init( drive, 3, 0x500 + canNodeIdIn );
    showerr( err, "initializing RPDO4" );

    err = tpdo1In.Init( drive, 0, 0x180 + canNodeIdIn );
    showerr( err, "initializing TPDO1" );

    err = tpdo2In.Init( drive, 1, 0x280 + canNodeIdIn );
    showerr( err, "initializing TPDO2" );

    err = tpdo3In.Init( drive, 2, 0x380 + canNodeIdIn );
    showerr( err, "initializing TPDO3" );

    err = tpdo4In.Init( drive, 3, 0x480 + canNodeIdIn );
    showerr( err, "initializing TPDO4" );

    err = drive.StartNode();
    showerr(err, "Going op");
}

void ProfilePositionModeMoveUsingSdos(Node& nodeIn, int targetPosIn, TPDO3& tpdo3In) 
{
    const Error* err = nodeIn.sdo.Dnld32(OBJID_PROFILE_POS, 0, targetPosIn);
    showerr(err, "setting the Target Position (0x607A)");
    uint16 ctrlWordVal = 0x001F;
    err = nodeIn.sdo.Dnld16(OBJID_CONTROL, 0, ctrlWordVal);
    showerr(err, "setting the Control Word (0x6040) to a value of 0x001F");
    ctrlWordVal = 0x000F;
    err = nodeIn.sdo.Dnld16(OBJID_CONTROL, 0, ctrlWordVal);
    showerr(err, "setting the Control Word (0x6040) to a value of 0x000F");
    int actualMotorPosition = 0;
    err = nodeIn.sdo.Upld32(OBJID_POS_ACT, 0, actualMotorPosition);
    showerr(err, "reading the actual motor position (0x6064) from Node 1");
    printf("Actual Position: %d\n", actualMotorPosition);
    int actualVelocity = 0;
    err = nodeIn.sdo.Upld32(0x606C, 0, actualVelocity);
    showerr(err, "reading the actual motor velocity (0x606C) from Node 1");
    printf("Actual Velocity: %d\n", actualVelocity);
    while( tpdo3In.actualPosition.Read() != targetPosIn )
    {
        // wait for actual position to equal target position
    }

    CML::Thread::sleep(100); // wait for 100ms for the node to settle into position
}

int main()
{
    const Error* err = 0;
    cml.SetDebugLevel(LOG_EVERYTHING);
    //cml.SetFlushLog(true);

    CopleyCAN hw;

    // Open the low-level CAN port
    printf("Opening card\n");

    err = hw.SetBaud(1000000);
    showerr(err, "Setting baud");

    CanOpen net;
    err = net.Open(hw);
    showerr(err, "Opening network");

    Node nonCopleyCanNode;
    int canNodeId = 1;
    TPDO1 tpdo1;
    TPDO2 tpdo2;
    TPDO3 tpdo3;
    TPDO4 tpdo4;
    RPDO1 rpdo1;
    RPDO2 rpdo2;
    RPDO3 rpdo3;
    RPDO4 rpdo4;
    
    InitNode( nonCopleyCanNode, net, canNodeId, rpdo1, rpdo2, rpdo3, rpdo4, tpdo1, tpdo2, tpdo3, tpdo4 );

    // Initialize a Copley CAN drive. The Copley node will generate the SYNC pulse. 
    // Use this for precise SYNC pulse generation. 
    Amp copleyNode;
    canNodeId = 2;
    AmpSettings ampSettingsObj;
    ampSettingsObj.synchPeriod = 100000; // 100ms SYNC period
    err = copleyNode.Init( net, canNodeId, ampSettingsObj );
    showerr( err, "initializing Copley drive" );

    // In the CAN log, I see this data being sent to the non-Copley node to enable it using RPDO1.
    err = rpdo1.Transmit( 0x0000 );
    showerr( err, "transmitting control word RPDO1" );
    err = rpdo1.Transmit( 0x0080 );
    showerr( err, "transmitting control word RPDO1" );
    err = rpdo1.Transmit( 0x0006 );
    showerr( err, "transmitting control word RPDO1" );
    err = rpdo1.Transmit( 0x0007 );
    showerr( err, "transmitting control word RPDO1" );
    err = rpdo1.Transmit( 0x000F );
    showerr( err, "transmitting control word RPDO1" );

    uint16 statusWordValue = 0;
    err = nonCopleyCanNode.sdo.Upld16(OBJID_STATUS, 0, statusWordValue);
    showerr(err, "reading the status word (0x6041) from Node 1");
    printf("Status Word: %d\n", statusWordValue);
    if ((statusWordValue & 0x0007)==0x0007) { "Node 1 is enabled, pushing current.\n"; }
    else { printf("Node 1 is not enabled.\n"); }

    int32 actualMotorPosition = 0; 
    err = nonCopleyCanNode.sdo.Upld32(OBJID_POS_ACT, 0, actualMotorPosition);
    showerr(err, "reading the actual motor position (0x6064) from Node 1");
    printf("Actual Position: %d\n", actualMotorPosition);

    // First, perform the homing routine.
    printf("Performing Homing Routine\n");
    int8 modeOfOp = 0x06;
    err = nonCopleyCanNode.sdo.Dnld8(OBJID_OP_MODE, 0, modeOfOp);
    showerr(err, "setting the mode of operation (0x6060) to 6 (homing mode) for Node 1");
    int8 homingMethod = 0x23;
    err = nonCopleyCanNode.sdo.Dnld8(OBJID_HOME_METHOD, 0, homingMethod);
    showerr(err, "setting the homing method (0x6098) to 0x23 for Node 1");

    uint16 ctrlWordVal = 0x0006;
    err = nonCopleyCanNode.sdo.Dnld16(OBJID_CONTROL, 0, ctrlWordVal);
    showerr(err, "setting the Control Word (0x6040) to a value of 0x0006");
    ctrlWordVal = 0x0007;
    err = nonCopleyCanNode.sdo.Dnld16(OBJID_CONTROL, 0, ctrlWordVal);
    showerr(err, "setting the Control Word (0x6040) to a value of 0x0007");
    ctrlWordVal = 0x000F;
    err = nonCopleyCanNode.sdo.Dnld16(OBJID_CONTROL, 0, ctrlWordVal);
    showerr(err, "setting the Control Word (0x6040) to a value of 0x000F");
    ctrlWordVal = 0x001F;
    err = nonCopleyCanNode.sdo.Dnld16(OBJID_CONTROL, 0, ctrlWordVal);
    showerr(err, "setting the Control Word (0x6040) to a value of 0x001F");

    CML::Thread::sleep(1000); // sleep for one second to let the homing routine complete.

    // Use SDO messages to make moves in profile position mode. Set to false to use PDO messages instead.
    bool useSdos = true;

    if (useSdos) 
    {
        // Next, perform the profile position mode move
        printf("Making Profile Position Mode Move\n");
        modeOfOp = 0x01;
        err = nonCopleyCanNode.sdo.Dnld8(OBJID_OP_MODE, 0, modeOfOp);
        showerr(err, "setting the mode of operation (0x6060) to 1 (profile position mode) for Node 1");
        ctrlWordVal = 0x0006;
        err = nonCopleyCanNode.sdo.Dnld16(OBJID_CONTROL, 0, ctrlWordVal);
        showerr(err, "setting the Control Word (0x6040) to a value of 0x0006");
        ctrlWordVal = 0x0007;
        err = nonCopleyCanNode.sdo.Dnld16(OBJID_CONTROL, 0, ctrlWordVal);
        showerr(err, "setting the Control Word (0x6040) to a value of 0x0007");
        ctrlWordVal = 0x000F;
        err = nonCopleyCanNode.sdo.Dnld16(OBJID_CONTROL, 0, ctrlWordVal);
        showerr(err, "setting the Control Word (0x6040) to a value of 0x000F");

        // In the CAN log, I see this series of moves being made 3 times.
        for (int i = 0; i < 3; i++)
        {
            ProfilePositionModeMoveUsingSdos(nonCopleyCanNode, 0x000186a0, tpdo3);
            ProfilePositionModeMoveUsingSdos(nonCopleyCanNode, 0x00030d40, tpdo3);
            ProfilePositionModeMoveUsingSdos(nonCopleyCanNode, 0x000493e0, tpdo3);
        }
    }
    // Use PDOs to move in profile position mode
    else 
    {
        // change mode of operation (0x6060) to 0x01 (profile position mode) using PDO
        rpdo2.Transmit(0x0000, 0x01);

        // enable the drive using the control word (0x6040)
        rpdo3.Transmit(0x0080, 0x00000000);
        rpdo3.Transmit(0x0080, 0x00000000);
        rpdo3.Transmit(0x0006, 0x00000000);
        rpdo3.Transmit(0x0007, 0x00000000);
        rpdo3.Transmit(0x000F, 0x00000000);

        // toggle bit 4 of the Control Word to start the move
        rpdo3.Transmit(0x000F, 0x00000000);
        rpdo3.Transmit(0x001F, 0x00000000);
        rpdo3.Transmit(0x000F, 0x00000000);

        // toggle bit 4 of the Control Word to start the move
        rpdo3.Transmit(0x000F, 0x000030F5);
        rpdo3.Transmit(0x001F, 0x000030F5);
        rpdo3.Transmit(0x000F, 0x000030F5);

        // toggle bit 4 of the Control Word to start the move
        rpdo3.Transmit(0x000F, 0x00006124);
        rpdo3.Transmit(0x001F, 0x00006124);
        rpdo3.Transmit(0x000F, 0x00006124);

        while (tpdo3.actualPosition.Read() != 0x00006124)
        {
            // wait for actual position to equal target position
        }
    }

    return 0;
}

// Just display the error (if there is one) and exit.
static void showerr(const Error* err, const char* msg)
{
    if (!err) return;
    printf("Error: %s - %s\n", msg, err->toString());
    exit(1);
}