/*

EcatCspMode.cpp

NOTE: Please use a real-time operating system when running
this example to ensure proper performance. If not using a
real-time OS, CSP mode is not recommended using CML. Instead,
use TwinCAT or another software that includes low-level drivers
for non-real-time operating systems.

The following example illustrates how to command a dual axis
EtherCAT drive in CSP mode using fixed PDO's.

CML will command positions in the positive direction until the
actual velocity exceeds the maximum velocity. It will then
slow down the motor and begin the same process in the negative
direction.

Fixed PDO's are handled by the drive's firmware in a high
priority thread. They are fixed, meaning their contents are
not changeable and are designed for use with the cyclic modes
of operation: CSP, CSV, CST, etc.

0x1B00 is the fixed TxPDO for CSP mode.
It relays the following important information back to CML
from the drive: Status Word (0x6041), Actual Position (0x6064),
Actual Velocity (0x606C), and Actual Torque (0x6077).

0x1700 is the fixed RxPDO for CSP mode.
It commands the following important objects to the drive:
Control Word (0x6040), Target Position (0x607A), Velocity
Offset (0x60B1), and Torque Offset (0x60B2).

*/

#include <CML.h>

#if defined( WIN32 )
#  include <ecat/ecat_winudp.h>
#else
#  include <ecat/ecat_linux.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <stdint.h>

CML_NAMESPACE_USE();

static void showerr(const Error* err, const char* msg);

int axisCt = 2;

int pdoUpdateRate = 3; // 3 millisecond PDO update rate

// This PDO represents the fixed transmit PDO (0x1B00) in the drive
class TPDO_NodeStat : public TPDO
{
public:
    Pmap16 statusWord;
    Pmap32 actualPos;
    Pmap32 followingErr;
    Pmap32 actualVel;
    Pmap16 actualTorque;
    int display;

    /// Default constructor for this PDO
    TPDO_NodeStat() { SetRefName("TPDO_Status"); }
    virtual ~TPDO_NodeStat() { KillRef(); }
    const Error* Init(Node& node, uint16 slot = 0x100)
    {
        // Set transmit type to transmit on events
        const Error* err = 0;// = SetType(255);

        int offset = (slot == 0x140) ? 0x800 : 0;

        // Let the various mapped variables know which 
        // objects in the amp's object dictionary they
        // are linked to.
        if (!err) err = statusWord.Init(0x6041 + offset, 0);
        if (!err) err = actualPos.Init(0x6064 + offset, 0);
        if (!err) err = followingErr.Init(0x60F4 + offset, 0);
        if (!err) err = actualVel.Init(0x606C + offset, 0);
        if (!err) err = actualTorque.Init(0x6077 + offset, 0);

        // Add the mapped variables
        if (!err) err = AddVar(statusWord);
        if (!err) err = AddVar(actualPos);
        if (!err) err = AddVar(followingErr);
        if (!err) err = AddVar(actualVel);
        if (!err) err = AddVar(actualTorque);

        // Program this PDO in the amp, and enable it
        if (!err) err = node.PdoSet(slot, *this);

        display = 0;

        return err;
    }
    virtual void Received(void)
    {
        uint16 s = statusWord.Read();
        int32 p = actualPos.Read();
        int32 e = followingErr.Read();
        int32 v = actualVel.Read();
        int16 t = actualTorque.Read();
        if (display)
            printf("stat: 0x%04x,  pos: %- 8d,  err: %- 8d,  vel: %- 8d,  trq: %- 5d \r", s, p, e, v, t);
    }
};

// This represents the fixed receive PDO used in CSP mode (0x1700)
class RPDO_NodeCtrl : public RPDO
{
    uint32 netRef;

public:
    Pmap16 ctrl;
    Pmap32 pos;
    Pmap32 voff;
    Pmap16 toff;

    /// Default constructor for this PDO
    RPDO_NodeCtrl() { SetRefName("RPDO_NodeCtrl"); }
    virtual ~RPDO_NodeCtrl() { KillRef(); }
    const Error* Init(Node& node, uint16 slot = 0x100)
    {
        netRef = node.GetNetworkRef();
        const Error* err = 0;

        // define an 0x800 offset for object indices belonging to axis B.
        int off = (slot == 0x140) ? 0x800 : 0;

        // Let the various mapped variables know which 
        // objects in the amp's object dictionary they
        // are linked to.
        if (!err) err = ctrl.Init(0x6040 + off, 0);
        if (!err) err = pos.Init(0x607A + off, 0);
        if (!err) err = voff.Init(0x60B1 + off, 0);
        if (!err) err = toff.Init(0x60B2 + off, 0);

        // Add the mapped variables
        if (!err) err = AddVar(ctrl);
        if (!err) err = AddVar(pos);
        if (!err) err = AddVar(voff);
        if (!err) err = AddVar(toff);

        // Program this PDO in the amp, and enable it
        if (!err) err = node.PdoSet(slot, *this);

        return err;
    }

    const Error* Send(uint16 C, int32 P, int32 vo = 0, int16 to = 0)
    {
        ctrl.Write(C);
        pos.Write(P);
        voff.Write(vo);
        toff.Write(to);

        RefObjLocker<Network> net(netRef);
        if (!net) return &NodeError::NetworkUnavailable;

        return Transmit(*net);
    }
};


/**
 */
int main(int argc, char** argv)
{
    const Error* err;
#if defined( WIN32 )
    WinUdpEcatHardware eth0("192.168.0.92");
#else
    LinuxEcatHardware eth0("eth0");
#endif

    cml.SetDebugLevel(LOG_EVERYTHING);
    cml.SetFlushLog(true);

    EtherCAT ecat;
    err = ecat.Open(eth0);
    showerr(err, "Opening EtherCAT network");

    Node node;

    printf("Initting amp\n");
    err = node.Init(ecat, -1);
    showerr(err, "Initting amp");

    printf("Setting up status PDO\n");
    TPDO_NodeStat statPDO[2];
    err = statPDO[0].Init(node);
    showerr(err, "Initting status PDO");

    printf("Setting up control PDO\n");
    RPDO_NodeCtrl ctrlPDO[2];
    err = ctrlPDO[0].Init(node);
    showerr(err, "Initting control PDO");

    printf("Setting mode\n");
    err = node.sdo.Dnld8(0x6060, 0, (int8)8);
    showerr(err, "Setting mode");


    err = node.sdo.Dnld32(0x6084, 0, (int32_t)16384); showerr(err, "Setting profile decel");
    err = node.sdo.Dnld32(0x6085, 0, (int32_t)65536); showerr(err, "Setting qstop decel");
    err = node.sdo.Dnld16(0x605D, 0, (int16_t)2); showerr(err, "Setting halt option");
    err = node.sdo.Dnld16(0x605A, 0, (int16_t)6); showerr(err, "Setting quickstop option");
    err = node.sdo.Dnld16(0x1C32, 1, (int16_t)2); showerr(err, "Setting sync mngr2 config to DC mode with SYNC0 event");
    err = node.sdo.Dnld16(0x1C33, 1, (int16_t)2); showerr(err, "Setting sync mngr3 config to DC mode with SYNC0 event");

    //Halt Option Code (0x605D):  1;2
    //Quick Stop Option Code (0x605A): 0;1;2;5;6

    printf("Setting PVT period to 3 * 10^-3 seconds = 3 milliseconds\n");
    err = node.sdo.Dnld8(0x60c2, 1, (int8)pdoUpdateRate);
    showerr(err, "Setting PVT period");
    err = node.sdo.Dnld8(0x60c2, 2, (int8)-3);
    showerr(err, "Setting PVT period");

    if (axisCt > 1)
    {
        printf("Setting up second axis\n");
        err = statPDO[1].Init(node, 0x140);
        showerr(err, "Initting status PDO axis 2");

        err = ctrlPDO[1].Init(node, 0x140);
        showerr(err, "Initting control PDO axis 2");

        err = node.sdo.Dnld8(0x6860, 0, (int8)8);
        showerr(err, "Setting mode");

        err = node.sdo.Dnld8(0x68c2, 1, (int8)pdoUpdateRate);
        showerr(err, "Setting PVT period");
        err = node.sdo.Dnld8(0x68c2, 2, (int8)-3);
        showerr(err, "Setting PVT period");
    }

    statPDO[1].display = 1;

    printf("Setting heartbeat\n");
    err = node.StartHeartbeat(100, 0);
    showerr(err, "Setting heartbeat");

    printf("Setting SYNC0\n");
    err = ecat.SetSync0Period(&node, 1000000 * pdoUpdateRate);
    showerr(err, "Setting SYNC0 period");

    printf("Starting node\n");
    err = node.StartNode();
    showerr(err, "Starting node");

    for (int i = 0; i < axisCt; i++) {
        err = ctrlPDO[i].Send(0x0080, 0);
        showerr(err, "setting bit 7 of control word");
    }

    printf("Press enter to move in csp mode\n");
    getchar();

    err = node.sdo.Dnld8(0x6060, 0, (int8)8);
    showerr(err, "Setting mode");

    int32 pos[2];
    printf("pos: ");
    for (int i = 0; i < axisCt; i++)
    {
        err = node.sdo.Upld32(0x6064 + 0x800 * i, 0, pos[i]);
        showerr(err, "Reading pos");
        printf("%d ", pos[i]);
    }
    printf("\n");

    for (int i = 0; i < axisCt; i++)
    {
        err = ctrlPDO[i].Send(0x000F, pos[i]);
        showerr(err, "Updating ctrl PDO");
    }

    int32_t cpr;
    err = node.sdo.Upld32(0x2383, 23, cpr);
    showerr(err, "Getting cts/rev");

    printf("\n\nCts/rev: %d\n\n", cpr);

    double vel = 0;      // cts/s
    double maxvel = cpr * 5;
    double acc = maxvel;

    int32_t wrap;
    err = node.sdo.Upld32(0x2220, 0, wrap);
    showerr(err, "Getting encoder wrap");

    int qstop = 0;
    int halt = 0;

    int i = 0;
    int delay = 0;
    while (1)
    {
        err = ecat.WaitCycleUpdate(100);
        showerr(err, "Waiting on cycle thread");

        for (int i = 0; i < axisCt; i++)
        {
            pos[i] += (int32)(vel * 0.001);

            if (wrap)
            {
                pos[i] %= wrap;
                if (pos[i] < 0) pos[i] += wrap;
            }

            int ctrl = 0x000f;
            if (qstop) ctrl = 0x0003;
            if (halt) ctrl |= 0x0100;

            if (statPDO[i].statusWord.Read() & 0x0008)
            {
                printf("\n\nClearing fault\n\n");
                ctrl |= 0x80;
            }

            err = ctrlPDO[i].Send(ctrl, pos[i]);
            showerr(err, "Updating ctrl PDO");
            //         printf( "Sending position %d, vel %f\n", pos[i], vel );

        }

        vel += acc * 0.001;

        if ((vel >= maxvel) && (acc > 0))
        {
            acc = 0;
            vel = maxvel;
            delay = 1000;
            printf("\nAt max velocity %f\n", vel);
        }

        //if( delay == 3000 )
        //{
        //   printf( "\nhalt\n" );
        //   halt = 1;
        //}
        //if( delay == 2000 ) qstop = 0;

        if (delay)
        {
            if (!--delay)
            {
                acc = -maxvel;
                printf("\nStarting slowdown\n");
            }
        }

        if ((vel <= -maxvel) && (acc < 0))
        {
            vel = -maxvel;
            acc = 0;
            printf("\nAt negative max velocity\n");
        }


        /*
              i++;
              if( i == 2000 ) printf( "\nSending shutdown\n" );
              if( i == 5000 ) printf( "\nClearing shutdown\n" );

              if( (i >= 2000) && (i<5000) )
                 ctrl &= ~1;
        */


        /*
           int cap = capPDO.cstat.Read();
           err = capSend.Send( cap );
           showerr( err, "Updating cap PDO" );
        */

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
