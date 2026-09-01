/*

CanFDProfilePositionMode.cpp

Multi-axis trapezoidal (Profile Position) moves over CAN-FD.

This example is a CAN-FD version of ProfilePositionMode.cpp.  It does two things
that a classic CAN example cannot do:

  1. It runs the data phase of every CAN message at 2 Mbps while arbitration
     stays at 1 Mbps.  CAN-FD switches bit rates in the middle of the frame
     (the BRS bit), so the bus can stay long and slow for arbitration while the
     payload flies by at the higher rate.

  2. It packs 20 bytes into a single RPDO.  Classic CAN tops out at 8 data bytes
     per frame, so a classic-CAN PDO can hold at most 8 bytes.  Copley plus
     drives running firmware 5.40 or later allow up to 32 bytes per PDO when
     CAN-FD is enabled, so the whole trapezoidal profile - target position,
     profile velocity, profile acceleration, profile deceleration - plus both
     control words fit in one frame:

     RPDO (20 bytes):
       [ Target Position (0x607A, 4)  ]
       [ Profile Velocity (0x6081, 4) ]
       [ Profile Accel (0x6083, 4)    ]
       [ Profile Decel (0x6084, 4)    ]
       [ Control Word (0x6040, 2) = 0x002F ]
       [ Control Word (0x6040, 2) = 0x003F ]

     The drive processes the mapped objects in the order they appear, so the
     new position AND the new profile shape land atomically, and the trailing
     0x002F -> 0x003F transition kicks off the updated trajectory.  On classic
     CAN this would take three separate frames plus two SDO writes.

     The example also maps a 16-byte TPDO back from each axis:

     TPDO (16 bytes):
       [ Position Actual (0x6064, 4) ]
       [ Velocity Actual (0x6069, 4) ]
       [ Position Error (0x60F4, 4)  ]
       [ Torque Actual (0x6077, 2)   ]
       [ Status Word (0x6041, 2)     ]

     20 and 16 are both legal CAN-FD frame lengths (the valid lengths above 8
     are 12, 16, 20, 24, 32, 48 and 64), so neither frame carries any padding.

     Position error is worth the extra four bytes when an axis stops short of
     its target: near zero means the trajectory itself ended there, and a large
     value means the trajectory is at the target and the servo is not following
     it.  On classic CAN there is no room for that distinction.

     The TPDO is synchronous, transmission type 1, so each drive sends it on
     every SYNC.  CML produces SYNC at a 10 ms period by default
     (AmpSettings::synchPeriod), which puts the status update at 10 ms.

  Each move waits for both axes to arrive before the next one is commanded, and
  the arrival is checked against the commanded target.  See WaitForMoves()
  below for why waiting takes two steps rather than a bare WaitMoveDone().

  ---------------------------------------------------------------------------
  BEFORE YOU RUN THIS: enable CAN-FD on the drive first
  ---------------------------------------------------------------------------

  CAN-FD is NOT backward compatible with classic CAN.  A classic-CAN-only node
  will actively destroy any CAN-FD frame it sees, and a CAN-FD-only network
  cannot be used to talk to a drive that has CAN-FD turned off.  So the drive
  has to be configured for CAN-FD before this program - or any other CAN-FD
  application - can reach it.

  Copley parameter 0x1B9 (CANopen object 0x21B7) sets the drive's CAN-FD data
  bit rate in bits/second.  It defaults to 0, which means "CAN-FD disabled".
  It is a flash parameter that the drive reads only at startup, so it has to be
  written to flash and the drive reset.  In CME:

      Tools > ASCII Command Line:   s f0x1b9 2000000

  then RESET OR POWER-CYCLE THE DRIVE.  Do this on every axis on the network.
  To go back to classic CAN, write 0 to the same parameter and reset.

  Requirements:
    - Copley plus drive (MP3 here) with firmware 5.40 or later.
    - Copley CAN card with 2.0 or later card firmware AND 2.0 or later driver.
      CopleyCAN::Open() returns CanError::NoFdSupport if you ask for CAN-FD on
      a card that can't do it.
    - EVERY node on the network must be CAN-FD capable.

*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>

#include "CML.h"
#include "can/can_copley.h"

using std::cout;

// If a namespace has been defined in CML_Settings.h, this
// macro starts using it.
CML_NAMESPACE_USE();

/* local functions */
static void showerr(const Error* err, const char* str);
static int  RunCanFdMove(void);

/**************************************************
 * Configuration
 **************************************************/

/// Number of axes.  A multi-axis MP3 shows up on CANopen as three
/// independent nodes, one per axis.
#define NUMBER_OF_AXES          3

/// Node ID of the first axis.  Axis N is at FIRST_NODE_ID + N.
#define FIRST_NODE_ID           1

/// Arbitration phase bit rate.  This is the classic CAN bit rate and it
/// is what the drive's CAN bit rate parameter must be set to.
#define CAN_ARBITRATION_BPS     1000000

/// CAN-FD data phase bit rate.  Setting this to 0 disables CAN-FD.
/// This must match Copley parameter 0x1B9 in the drive (see the header
/// comment).  Note that standard CAN transceivers are only rated to 1 Mbps,
/// so the usable data rate has to be evaluated per network.  2 Mbps works on
/// a short bus with only CAN-FD capable devices on it.
#define CAN_FD_DATA_BPS         2000000

/// PDO slots.  CML claims slots 0 and 1 for its own use during Amp::Init(),
/// so slots 2 and 3 are free for the application.
#define RPDO_SLOT               2
#define TPDO_SLOT               3

/// TPDO transmission type.  1 to 240 is synchronous: the drive sends the PDO
/// on every Nth SYNC.  CML's default SYNC period is 10 ms, so 1 gives a
/// status update every 10 ms.
#define TPDO_SYNC_COUNT         1

/**************************************************
 * Motion parameters (user units)
 **************************************************/
#define MOVE_COUNT              10
#define MOVE_DISTANCE           500
#define BASE_PROFILE_VEL        50000
#define BASE_PROFILE_ACC        50000
#define BASE_PROFILE_DEC        50000

/// How long to allow for the drive to report that the new set-point was
/// accepted, in milliseconds.  A move of zero length never reports a start,
/// so a timeout here is not an error.
#define MOVE_START_TIMEOUT_MS   500

/// How long to allow a single move to complete, in milliseconds.  Size this
/// for the longest move in the list plus margin - the last move here is
/// 85,000 counts, which at 68,000 counts/s takes a bit over a second.
#define MOVE_TIMEOUT_MS         10000

/// How close to the target the axis has to settle, in encoder counts, for the
/// move to count as good.  The drive's own "target reached" window (objects
/// 0x6067 / 0x6068) is what actually ends the move; this is an independent
/// check on the position the TPDO brings back.
#define POSITION_TOLERANCE      50

/***************************************************************************/
/**
Receive PDO carrying a complete trapezoidal move: target position, the full
profile shape, and the two control word writes that trigger it.

This PDO is 20 bytes long, which requires CAN-FD.  Attempting to map this much
data into a PDO on a drive that does not have CAN-FD enabled will fail with an
SDO abort when the mapping is downloaded.
*/
/***************************************************************************/
class RpdoProfileMove : public RPDO
{
    uint32 networkReference;  // used to transmit the RPDO on the network

    Pmap32 targetPosition;    // 0x607A  target position       (4 bytes)
    Pmap32 profileVelocity;   // 0x6081  profile velocity      (4 bytes)
    Pmap32 profileAccel;      // 0x6083  profile acceleration  (4 bytes)
    Pmap32 profileDecel;      // 0x6084  profile deceleration  (4 bytes)
    Pmap16 controlWord1;      // 0x6040  control word          (2 bytes)
    Pmap16 controlWord2;      // 0x6040  control word          (2 bytes)
                              //                        total = 20 bytes

public:
    RpdoProfileMove() {}

    /// Initialize and program the RPDO.
    /// @param amp        the amp object for this axis
    /// @param canId      node ID of this axis
    /// @param slotNumber PDO slot to use (CML uses 0 and 1)
    const Error* Init(Amp& amp, int canId, int slotNumber);

    /// Transmit a complete move.  All values are in the drive's internal
    /// (load) units - use Amp::PosUser2Load(), Amp::VelUser2Load() and
    /// Amp::AccUser2Load() to convert from user units.
    const Error* Transmit(int32 posLoad, int32 velLoad, int32 accLoad, int32 decLoad);

    /// Number of data bytes this PDO puts on the wire.
    int GetByteCt(void) { return GetBitCt() >> 3; }
};

/***************************************************************************/
/**
Transmit PDO carrying live axis feedback.  16 bytes, which also requires
CAN-FD.  The drive sends this on every Nth SYNC.
*/
/***************************************************************************/
class TpdoAxisStatus : public TPDO
{
    Pmap32 positionActual;    // 0x6064  position actual value (4 bytes)
    Pmap32 velocityActual;    // 0x6069  velocity actual value (4 bytes)
    Pmap32 positionError;     // 0x60F4  following error       (4 bytes)
    Pmap16 torqueActual;      // 0x6077  torque actual value   (2 bytes)
    Pmap16 statusWord;        // 0x6041  status word           (2 bytes)
                              //                        total = 16 bytes

    uint32 rxCount;

public:
    TpdoAxisStatus() { rxCount = 0; }

    /// Initialize and program the TPDO.
    /// @param syncCount the drive sends this PDO on every syncCount'th SYNC
    const Error* Init(Amp& amp, int canId, int slotNumber, byte syncCount);

    /// Called by the CML network read thread every time this PDO arrives.
    /// Keep this short - it runs on the receive thread.
    virtual void Received(void) { rxCount++; }

    uint32 GetRxCount(void) { return rxCount; }
    int32  GetPositionActual(void) { return positionActual.Read(); }
    int32  GetVelocityActual(void) { return velocityActual.Read(); }
    int32  GetPositionError(void) { return positionError.Read(); }
    int16  GetTorqueActual(void) { return torqueActual.Read(); }
    uint16 GetStatusWord(void) { return (uint16)statusWord.Read(); }

    /// Number of data bytes this PDO puts on the wire.
    int GetByteCt(void) { return GetBitCt() >> 3; }
};

/***************************************************************************/
/**
Wait for every axis to finish the move that was just commanded, and record
how long each one took.

Two things make this more than a bare Amp::WaitMoveDone() call.

First, AMPEVENT_MOVEDONE is still set from the PREVIOUS move at the instant
the RPDO goes out - the drive has not had a chance to report the new set-point
yet - so waiting on it straight away returns immediately on the stale event and
the program runs on while the axis is still moving.  Each axis is therefore
tracked through two states: MOVEDONE has to go clear (the drive accepted the
set-point and the move is under way) before the wait for it to set again means
anything.

Second, the axes are polled together rather than waited on one after another.
Blocking on axis 0 and then on axis 1 would fold the first axis's travel time
into the second's, which hides the case where one axis is much slower than the
other.  Amp::GetEventMask() is a cheap read of the event mask CML maintains
from each drive's status PDO, so polling it costs nothing on the network.

@param amp      Array of amp objects
@param axisCt   Number of axes
@param elapsed  Filled in with each axis's move time in milliseconds
@param arrived  Filled in with true for each axis that finished in time
@param startTo  How long to allow for the move to start, in ms
@param moveTo   How long to allow for the move to finish, in ms
@return An error object if an axis faulted, otherwise NULL.  A move that
        simply did not finish in time is reported through 'arrived', not as
        an error, so the caller can report every axis before giving up.
*/
/***************************************************************************/
static const Error* WaitForMoves(Amp amp[], int axisCt, uint32 elapsed[],
    bool arrived[], Timeout startTo, Timeout moveTo)
{
    // Anything here means the move will never complete.
    const AMP_EVENT failEvents = (AMP_EVENT)(AMPEVENT_NODEGUARD | AMPEVENT_FAULT |
        AMPEVENT_ERROR | AMPEVENT_DISABLED | AMPEVENT_QUICKSTOP | AMPEVENT_ABORT);

    bool started[NUMBER_OF_AXES];
    int  remaining = axisCt;

    for (int i = 0; i < axisCt; i++)
    {
        started[i] = false;
        arrived[i] = false;
        elapsed[i] = 0;
    }

    uint32 t0 = Thread::getTimeMS();

    while (remaining)
    {
        uint32 ms = Thread::getTimeMS() - t0;

        for (int i = 0; i < axisCt; i++)
        {
            if (arrived[i]) continue;

            AMP_EVENT ev;
            const Error* err = amp[i].GetEventMask(ev);
            if (err) return err;

            if (ev & failEvents)
                return amp[i].GetErrorStatus();

            if (!started[i])
            {
                // The move has started once the drive clears "move done".
                if (!(ev & AMPEVENT_MOVEDONE))
                    started[i] = true;

                // A zero length move never clears it.  Treat the axis as
                // already there once the start window has passed.
                else if ((Timeout)ms >= startTo)
                {
                    started[i] = true;
                    arrived[i] = true;
                    elapsed[i] = ms;
                    remaining--;
                }
            }
            else if (ev & AMPEVENT_MOVEDONE)
            {
                arrived[i] = true;
                elapsed[i] = ms;
                remaining--;
            }
        }

        if (!remaining) break;

        if ((Timeout)(Thread::getTimeMS() - t0) >= moveTo)
        {
            // Out of time.  Record what each unfinished axis managed and let
            // the caller report it.
            for (int i = 0; i < axisCt; i++)
                if (!arrived[i]) elapsed[i] = Thread::getTimeMS() - t0;
            break;
        }

        Thread::sleep(2);
    }

    return 0;
}

/***************************************************************************/
/**
Program entry point. Run the dual-axis CAN-FD move.
*/
/***************************************************************************/
int main(void)
{
    // The libraries define one global object of type
    // CopleyMotionLibraries named cml.
    //
    // This object has a couple handy member functions
    // including this one which enables the generation of
    // a log file for debugging
    cml.SetDebugLevel(LOG_EVERYTHING);
    //cml.SetFlushLog( true );

    return RunCanFdMove();
}

/***************************************************************************/
/**
The actual CAN-FD demonstration: dual-axis trapezoidal moves driven by one
20 byte RPDO per axis, with a 12 byte status TPDO coming back.
*/
/***************************************************************************/
static int RunCanFdMove(void)
{
    printf("--- Multi-axis Profile Position over CAN-FD ---\n\n");

    // Everything that talks to the network lives in this one scope, declared
    // in dependency order.  C++ destroys locals in reverse order of
    // declaration, so teardown runs PDOs -> amps -> network -> CAN interface:
    // every object is gone before the thing it depends on.  Declaring the amps
    // or the PDOs at file scope instead (as some of the older examples do)
    // outlives the network object and tears down in the wrong order.
    CopleyCAN       hw("CAN0");
    CanOpen         net;
    Amp             amp[NUMBER_OF_AXES];
    RpdoProfileMove rpdoProfileMove[NUMBER_OF_AXES];
    TpdoAxisStatus  tpdoAxisStatus[NUMBER_OF_AXES];

    // Arbitration phase bit rate - same as classic CAN.
    const Error* err = hw.SetBaud(CAN_ARBITRATION_BPS);
    showerr(err, "Setting CAN arbitration bit rate");

    // Data phase bit rate.  This enables CAN-FD.  It must be called before
    // Open() - the card only picks the setting up when the port is opened,
    // and SetFdBaud() returns CanError::AlreadyOpen if the port is already up.
    err = hw.SetFdBaud(CAN_FD_DATA_BPS);
    showerr(err, "Setting CAN-FD data bit rate");

    printf("CAN-FD: %d bps arbitration, %d bps data\n\n",
        CAN_ARBITRATION_BPS, CAN_FD_DATA_BPS);

    err = net.Open(hw);

    // If the card firmware or the driver is older than 2.0 the card cannot do
    // CAN-FD and Open() fails here rather than silently falling back.
    if (err == &CanError::NoFdSupport)
    {
        printf("This CAN card does not support CAN-FD.\n");
        printf("CAN-FD needs card firmware 2.0 or later and driver 2.0 or later.\n");
        return 1;
    }
    showerr(err, "Opening CANopen network");

    for (int i = 0; i < NUMBER_OF_AXES; i++)
    {
        int nodeId = FIRST_NODE_ID + i;

        err = amp[i].Init(net, nodeId);
        showerr(err, "Initting amp");

        err = amp[i].PreOpNode();
        showerr(err, "Preopping node");

        // The 20 byte RPDO.  This mapping is only accepted by the drive
        // because CAN-FD is enabled - classic CAN caps a PDO at 8 bytes.
        err = rpdoProfileMove[i].Init(amp[i], nodeId, RPDO_SLOT);
        showerr(err, "Initting profile move RPDO");

        // The 12 byte status TPDO.
        err = tpdoAxisStatus[i].Init(amp[i], nodeId, TPDO_SLOT, TPDO_SYNC_COUNT);
        showerr(err, "Initting axis status TPDO");

        printf("Axis %d (node %d): RPDO %d bytes, TPDO %d bytes\n",
            i, nodeId,
            rpdoProfileMove[i].GetByteCt(),
            tpdoAxisStatus[i].GetByteCt());
    }

    // Start all nodes.
    for (int i = 0; i < NUMBER_OF_AXES; i++)
    {
        err = amp[i].StartNode();
        showerr(err, "Starting node");
    }

    // Put every axis in CANopen profile mode with a trapezoidal profile.
    // The profile velocity / accel / decel values that would normally be set
    // here by SDO are instead sent with every RPDO below, so there is no need
    // to pre-load them.
    int16 trapezoidalProfile = 0;
    for (int i = 0; i < NUMBER_OF_AXES; i++)
    {
        err = amp[i].SetAmpMode(AMPMODE_CAN_PROFILE);
        showerr(err, "Setting amp mode");

        err = amp[i].sdo.Dnld16(OBJID_PROFILE_TYPE, 0, trapezoidalProfile);
        showerr(err, "Setting profile type");
    }

    printf("\nRunning %d moves per axis...\n\n", MOVE_COUNT);

    int    polarity = 1;
    int    failures = 0;
    int32  targetLoad[NUMBER_OF_AXES];
    uint32 elapsed[NUMBER_OF_AXES];
    bool   arrived[NUMBER_OF_AXES];

    for (int m = 0; m < MOVE_COUNT; m++)
    {
        if (m % 2) { polarity *= -1; }

        // Vary the profile shape from move to move.  On classic CAN each of
        // these would need its own SDO write; here they ride along in the
        // same frame as the target position.
        uunit targetPos = (uunit)(m * MOVE_DISTANCE * polarity);
        uunit vel = (uunit)(BASE_PROFILE_VEL + m * 2000);
        uunit acc = (uunit)(BASE_PROFILE_ACC + m * 2000);
        uunit dec = (uunit)(BASE_PROFILE_DEC + m * 2000);

        for (int i = 0; i < NUMBER_OF_AXES; i++)
        {
            // The PDO carries raw drive units, so convert from user units
            // here.  Amp::SetProfileVel() and friends do this internally when
            // they go out over SDO.
            targetLoad[i] = amp[i].PosUser2Load(targetPos);

            err = rpdoProfileMove[i].Transmit(
                targetLoad[i],
                amp[i].VelUser2Load(vel),
                amp[i].AccUser2Load(acc),
                amp[i].AccUser2Load(dec));
            showerr(err, "Sending profile move PDO");
        }

        printf("move %2d: target %8d  vel %7d  acc %7d  dec %7d\n",
            m, (int)targetPos, (int)vel, (int)acc, (int)dec);

        // Wait for both axes to get there before commanding the next move.
        // Without this the next target overwrites the current one mid-flight
        // (which is what dynamic trajectory updating is for, but it makes the
        // final positions impossible to check).
        err = WaitForMoves(amp, NUMBER_OF_AXES, elapsed, arrived,
            MOVE_START_TIMEOUT_MS, MOVE_TIMEOUT_MS);
        showerr(err, "Waiting for move to complete");

        for (int i = 0; i < NUMBER_OF_AXES; i++)
        {
            int32 posErr = tpdoAxisStatus[i].GetPositionActual() - targetLoad[i];
            bool  inWindow = (posErr <= POSITION_TOLERANCE) && (-posErr <= POSITION_TOLERANCE);
            bool  ok = arrived[i] && inWindow;

            if (!ok) failures++;

            // 'err' is how far the axis is from the target this program
            // asked for.  'ferr' is the drive's own following error - the gap
            // between its trajectory and the motor.  If an axis stops short
            // with err large and ferr near zero, its trajectory generator
            // ended early (a drive limit); if ferr is large too, the servo is
            // not following the trajectory (tuning or current limit).
            printf("         axis %d: pos %8d  err %7d  ferr %7d  %5u ms  status 0x%04x  %s\n",
                i,
                (int)tpdoAxisStatus[i].GetPositionActual(),
                (int)posErr,
                (int)tpdoAxisStatus[i].GetPositionError(),
                (unsigned)elapsed[i],
                (unsigned)tpdoAxisStatus[i].GetStatusWord(),
                ok ? "ok" : (arrived[i] ? "OUT OF TOLERANCE" : "DID NOT ARRIVE"));
        }
    }

    printf("\nMoves finished.\n");
    for (int i = 0; i < NUMBER_OF_AXES; i++)
        printf("Axis %d received %u status TPDOs of %d bytes each.\n",
            i, (unsigned)tpdoAxisStatus[i].GetRxCount(),
            tpdoAxisStatus[i].GetByteCt());

    if (failures)
        printf("\n%d of %d axis moves did not reach target.\n",
            failures, MOVE_COUNT * NUMBER_OF_AXES);
    else
        printf("\nAll %d axis moves reached target within %d counts.\n",
            MOVE_COUNT * NUMBER_OF_AXES, POSITION_TOLERANCE);

    return failures ? 1 : 0;
}

/***************************************************************************/
/**
Initialize the 20 byte profile move RPDO and program it into the drive.
@param amp The amp object for this axis
@param canId Node ID for the drive
@param slotNumber The PDO slot to use.  CML uses slots 0 and 1.
@return An error object, or NULL on success
*/
/***************************************************************************/
const Error* RpdoProfileMove::Init(Amp& amp, int canId, int slotNumber)
{
    networkReference = amp.GetNetworkRef();

    // Init the base class.  Slots 0-3 use 11 bit CAN message IDs.
    uint32 canMessageId = 0x200 + slotNumber * 0x100 + canId;
    const Error* err = RPDO::Init(canMessageId);

    // Init the mapping objects that describe the data mapped to this PDO.
    if (!err) err = targetPosition.Init(OBJID_PROFILE_POS);
    if (!err) err = profileVelocity.Init(OBJID_PROFILE_VEL);
    if (!err) err = profileAccel.Init(OBJID_PROFILE_ACC);
    if (!err) err = profileDecel.Init(OBJID_PROFILE_DEC);
    if (!err) err = controlWord1.Init(OBJID_CONTROL);
    if (!err) err = controlWord2.Init(OBJID_CONTROL);

    // Add these variables to the PDO.  Order matters - the drive acts on the
    // mapped objects in the order they appear, so the two control words have
    // to come last, after the new profile has landed.
    if (!err) err = AddVar(targetPosition);
    if (!err) err = AddVar(profileVelocity);
    if (!err) err = AddVar(profileAccel);
    if (!err) err = AddVar(profileDecel);
    if (!err) err = AddVar(controlWord1);
    if (!err) err = AddVar(controlWord2);

    // Load the control word values so that every transmission of this PDO
    // starts a new move: 0x002F, then 0x003F sets the "new set-point" bit.
    controlWord1.Write((int16)0x002F);
    controlWord2.Write((int16)0x003F);

    // Set the PDO type so that its data will be acted on immediately.
    if (!err) err = SetType(255);

    // Program this PDO into the drive.
    if (!err) err = amp.PdoSet(slotNumber, *this);

    return err;
}

/***************************************************************************/
/**
Transmit a complete move in a single 20 byte CAN-FD frame.
@param posLoad Target position, drive units (counts)
@param velLoad Profile velocity, drive units
@param accLoad Profile acceleration, drive units
@param decLoad Profile deceleration, drive units
@return An error object, or NULL on success
*/
/***************************************************************************/
const Error* RpdoProfileMove::Transmit(int32 posLoad, int32 velLoad,
    int32 accLoad, int32 decLoad)
{
    targetPosition.Write(posLoad);
    profileVelocity.Write(velLoad);
    profileAccel.Write(accLoad);
    profileDecel.Write(decLoad);

    // Acquire a reference to the network.
    RefObjLocker<Network> net(networkReference);

    // Check that the network is available.
    if (!net) return &NodeError::NetworkUnavailable;

    // Transmit the RPDO on the network.
    return RPDO::Transmit(*net);
}

/***************************************************************************/
/**
Initialize the 16 byte axis status TPDO and program it into the drive.

The PDO is synchronous rather than event driven. Synchronous transmission uses 
the SYNC message CML is already producing and gives a deterministic update rate.

@param amp The amp object for this axis
@param canId Node ID for the drive
@param slotNumber The PDO slot to use.  CML uses slots 0 and 1.
@param syncCount The drive sends this PDO on every syncCount'th SYNC (1-240)
@return An error object, or NULL on success
*/
/***************************************************************************/
const Error* TpdoAxisStatus::Init(Amp& amp, int canId, int slotNumber,
    byte syncCount)
{
    // Init the base class.  Slots 0-3 use 11 bit CAN message IDs.
    uint32 canMessageId = 0x180 + slotNumber * 0x100 + canId;
    const Error* err = TPDO::Init(canMessageId);

    if (!err) err = positionActual.Init(OBJID_POS_ACT);
    if (!err) err = velocityActual.Init(OBJID_VEL_ACT);
    if (!err) err = positionError.Init(OBJID_POS_ERR);
    if (!err) err = torqueActual.Init(OBJID_TORQUE_ACTUAL);
    if (!err) err = statusWord.Init(OBJID_STATUS);

    if (!err) err = AddVar(positionActual);
    if (!err) err = AddVar(velocityActual);
    if (!err) err = AddVar(positionError);
    if (!err) err = AddVar(torqueActual);
    if (!err) err = AddVar(statusWord);

    // Transmission types 1 to 240 are synchronous: send on every Nth SYNC.
    if (!err) err = SetType(syncCount);

    // Program this PDO into the drive and enable it.
    if (!err) err = amp.PdoSet(slotNumber, *this);

    return err;
}

/***************************************************************************/
/**
Show any errors to the user.
*/
/***************************************************************************/
static void showerr(const Error* err, const char* str)
{
    if (err)
    {
        printf("Error %s: %s\n", str, err->toString());
        exit(1);
    }
}
