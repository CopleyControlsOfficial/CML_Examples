/*

BeckhoffAsicChip.cpp

Drive a non-CoE EtherCAT I/O module through PDOs, so the cycle thread
carries its process data at the cycle rate.

REQUIRES THE BUILD CONTAINING CML 2.20.03.  Three things here need it:
EtherCAT::GetSyncMgrInfo; the fact that an input sync manager is configured
at all (before CML-130 the transmit lists were never initialized, so an
input sync manager's registers read back as zeros and an input PDO had
nowhere to land); and the input sync manager carrying the device's own
length rather than one byte.

WHY PDOs AND NOT NodeRead/NodeWrite

Once the node is in SAFE-OP or OPERATIONAL, CML owns its output sync
manager: the cycle thread writes that list's buffer there every period.
An application writing the same address with NodeWrite is in a race with
it and loses, every cycle.  Keeping the node in PRE-OP avoids the race -
the cycle thread skips a node that is not in safe-op or OP - but then the
sync manager watchdog is not being fed either, and on a module whose
output channel has the watchdog bit set in its control byte (0x44, bit 6)
that bounds how long acyclic writes can work at all.

PDOs are not a tidier way to do the same thing.  On this device class they
are the only way that holds.

WHAT IT DOES NOT HANDLE

A process-data channel wider than two bytes.  Pmap8 and Pmap16 cover one
and two; a wider channel wants several variables per PDO, and the right
split depends on what the device's channels actually are.  It says so and
stops rather than guessing.

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

CML_NAMESPACE_USE();

/* local functions */
static void showerr(const Error* err, const char* str);

/* local data */
int16 etherCatNodeID = 1002;        // EtherCAT node ID (alias)

/* An ESC has at most eight sync managers. */
static const int MAX_SM = 8;

/* How long the demonstration loop at the bottom runs, and how long it
   waits between steps.  Guarded so this file can be compiled against a
   simulated device and executed - which is how the missing Transmit above
   would have been caught before it reached a bench. */
#ifndef BECKHOFF_PDO_STEPS
#define BECKHOFF_PDO_STEPS      30
#endif
#ifndef BECKHOFF_PDO_STEP_MS
#define BECKHOFF_PDO_STEP_MS  1000
#endif

   /*--------------------------------------------------------------------------
     THE ONE THING YOU MAY HAVE TO SET.

     Width in bytes of each process-data channel, by ESC sync manager number.
     Zero means "take it from the device", which is what you want whenever
     the device's EEPROM declares a length.

     Set an entry only when this program tells you to - it prints a line per
     channel whose length the EEPROM did not declare, naming the sync manager
     and the width it is falling back to.  On the bench module, whose EEPROM
     declares nothing, the two output channels are one byte each and correct
     by the fallback, and the input channel is two:

         static const uint16 channelWidth[MAX_SM] = { 0, 0, 2, 0, 0, 0, 0, 0 };
                                                            ^ SM2, the inputs
    --------------------------------------------------------------------------*/
static const uint16 channelWidth[MAX_SM] = { 0, 0, 0, 0, 0, 0, 0, 0 };

class BeckhoffAsicChip : public Node {};

struct Channel
{
    int    sm;          // ESC sync manager number
    uint16 width;       // bytes this program maps
    bool   isOutput;    // data flows master to slave
    int    listIndex;   // what to put in Node::smIndex
};

/* The sync manager length register, for the confirming read below:
   0x0800 + 8 * n, length at +2. */
static uint16 SmLenRegister(int sm)
{
    return (uint16)(0x0800 + 8 * sm + 2);
}

int main(void)
{
    cml.SetDebugLevel(LOG_EVERYTHING);

#if defined( WIN32 )
    WinUdpEcatHardware hw("192.168.0.33");
#else
    LinuxEcatHardware hw("eth0");
#endif

    EtherCAT net;
    EtherCatSettings ecatSettings;
    ecatSettings.cyclePeriod = 20;

    const Error* err = net.Open(hw, ecatSettings);
    showerr(err, "Opening network");

    BeckhoffAsicChip ioModule;

    /* Node::Init, not EtherCAT::AttachNode.  Init grabs the network
       reference first, and the sync manager configuration needs it. */
    err = ioModule.Init(net, etherCatNodeID);
    showerr(err, "initializing the I/O module");

    cout << "Node Alias: " << ioModule.GetNodeID() << endl;

    /*-----------------------------------------------------------------------
      STEP 1.  Ask the device what it has.

      GetSyncMgrInfo answers from what CML read out of this device's EEPROM,
      so it reports the EEPROM's address, control byte and declared length -
      and separately the length CML configured.  A channel it refuses is one
      that carries no process data: a mailbox, or one this device does not
      have.
     -----------------------------------------------------------------------*/
    Channel found[MAX_SM];
    int foundCt = 0;
    int guessed = 0;

    cout << endl << "Process data channels this device reports:" << endl;

    for (int sm = 0; sm < MAX_SM; sm++)
    {
        EcatSyncMgrInfo info;

        if (net.GetSyncMgrInfo(&ioModule, sm, info)) continue;

        Channel& c = found[foundCt];
        c.sm = sm;
        c.isOutput = info.isOutput;
        c.listIndex = info.listIndex;

        /* Where the width comes from, in order: what you set above, then the
           EEPROM, then CML's fallback. */
        const char* source;

        if (channelWidth[sm])
        {
            c.width = channelWidth[sm];
            source = "set in channelWidth[]";
        }
        else if (info.eepromLen)
        {
            c.width = info.eepromLen;
            source = "from the device's EEPROM";
        }
        else
        {
            c.width = info.cfgLen;
            source = "CML's fallback - THE DEVICE DID NOT SAY";
            guessed++;
        }

        printf("  SM%d: 0x%04x  ctrl 0x%02x  %-7s  %s list index %d  "
            "%u byte(s), %s\n",
            sm, info.ramAddr, info.smCtrl,
            c.isOutput ? "output" : "input",
            c.isOutput ? "receive" : "transmit", c.listIndex,
            c.width, source);

        foundCt++;
    }

    if (!foundCt)
    {
        cout << endl << "No process-data sync managers found.  Either this "
            "node is not attached" << endl
            << "or its EEPROM carries no SyncManager category - the cml.log "
            "dump for this" << endl
            << "node is where to look." << endl;
        return 1;
    }

    if (guessed)
    {
        printf("\n%d channel(s) above carry no length in this device's "
            "EEPROM, so the width\nshown is CML's fallback and not the "
            "device's answer.  If the module's image\nis wider than that, "
            "set the real width in channelWidth[] at the top of this\n"
            "file, by sync manager number, and run it again.  cml.log "
            "carries the same\nwarning per channel.\n", guessed);
    }

    /*-----------------------------------------------------------------------
      STEP 2.  One PDO per channel, sized to that channel.

      The object index and sub-index given to Pmap::Init are labels only on
      a device with no mailbox - there is no object dictionary to look them
      up in.  SetVerifyFixedPdoMapping( false ) is required for the same
      reason: verification reads the node's mapping objects over SDO, and
      there is no SDO here.

      The PDO and Pmap objects have to outlive this scope, so they are plain
      arrays rather than anything allocated in the loop.  Both widths are
      declared per slot and only the matching one is used, which costs a few
      bytes and removes every question about lifetime.
     -----------------------------------------------------------------------*/
    Pmap8  var8[MAX_SM];
    Pmap16 var16[MAX_SM];
    RPDO   rpdo[MAX_SM];
    TPDO   tpdo[MAX_SM];

    for (int i = 0; i < foundCt; i++)
    {
        Channel& f = found[i];

        if (f.width != 1 && f.width != 2)
        {
            printf("\nSM%d is %u bytes.  This example maps one 8- or 16-bit "
                "variable per sync\nmanager, so it cannot map this one - a "
                "wider channel needs a mapping that\nmatches what the "
                "device's channels actually are.\n", f.sm, f.width);
            return 1;
        }

        Pmap* var;

        if (f.width == 1)
        {
            err = var8[i].Init(f.isOutput ? 0x7000 : 0x6000, 1);
            var = &var8[i];
        }
        else
        {
            err = var16[i].Init(f.isOutput ? 0x7000 : 0x6000, 1);
            var = &var16[i];
        }
        showerr(err, "initializing a process data mapping");

        ioModule.smIndex = f.listIndex;

        if (f.isOutput)
        {
            rpdo[i].SetVerifyFixedPdoMapping(false);
            err = rpdo[i].AddVar(*var);
            showerr(err, "adding a variable to a receive PDO");

            err = ioModule.PdoSet(0, rpdo[i], true);
            showerr(err, "mapping a receive PDO");
        }
        else
        {
            tpdo[i].SetVerifyFixedPdoMapping(false);
            err = tpdo[i].AddVar(*var);
            showerr(err, "adding a variable to a transmit PDO");

            err = ioModule.PdoSet(0, tpdo[i], true);
            showerr(err, "mapping a transmit PDO");
        }

        /*-------------------------------------------------------------------
          STEP 3.  And confirm the mapping landed on the sync manager it was
          meant for, by reading the length back.

          GetSyncMgrInfo is one answer; this is an independent second one,
          and it costs one register read.  If the PDO went somewhere else
          this length will not be the width just mapped.
         -------------------------------------------------------------------*/
        uint16 len = 0;
        err = net.NodeRead(&ioModule, (int16)SmLenRegister(f.sm), 2, &len);
        showerr(err, "reading a sync manager's length back");

        if (len != f.width)
        {
            printf("\n*** The PDO did not land on SM%d. ***\n"
                "A %u-byte variable was mapped and the sync manager reads "
                "%u.\nThe mapping went to a different sync manager, which "
                "means receive/transmit\nlist index %d is not SM%d's - and "
                "that came from EtherCAT::GetSyncMgrInfo,\nso the cml.log "
                "EEPROM dump for this node is where to look.\n",
                f.sm, f.width, len, f.listIndex, f.sm);
            return 1;
        }

        printf("  SM%d mapped: %u byte(s), confirmed in the register\n",
            f.sm, len);
    }

    err = ioModule.PreOpNode();
    showerr(err, "Preopping node");

    err = ioModule.StartNode();
    showerr(err, "Starting node");

    printf("\nI/O module initialized: %d process-data channel(s) mapped.\n",
        foundCt);

    /*-----------------------------------------------------------------------
      From here on the I/O is local.  Writing an output is an assignment
      followed by a Transmit, which the next cyclic frame carries and every
      frame after it repeats; reading an input is a read of whatever the
      last frame brought back.  No datagram from this thread, nothing racing
      CML's cyclic write to the same address, and the sync manager watchdog
      is fed by the cyclic traffic.
     -----------------------------------------------------------------------*/
    int bit = 0;

    /* How many output bits there are in total, across however many output
       channels this device has. */
    int outputBits = 0;
    for (int i = 0; i < foundCt; i++)
        if (found[i].isOutput) outputBits += 8 * found[i].width;

    if (!outputBits)
        cout << "This device has no output channels - inputs only." << endl;

    int count = 0;
    while (1)
    {
        /* Walk one output at a time, across all the output channels in the
           order the device reports them. */
        int seen = 0;

        for (int i = 0; i < foundCt; i++)
        {
            if (!found[i].isOutput) continue;

            const int bits = 8 * found[i].width;
            const int32 val = (bit >= seen && bit < seen + bits)
                ? ((int32)1 << (bit - seen)) : 0;

            if (found[i].width == 1) var8[i].Write((byte)val);
            else                     var16[i].Write((int16)val);

            /* THE PUBLISH.  Without this the frame keeps carrying whatever
               the Pmap held when the PDO was mapped - which is zero */
            err = rpdo[i].Transmit(net);
            showerr(err, "publishing an output PDO");

            seen += bits;
        }

        if (outputBits && ++bit >= outputBits) bit = 0;

        /* Report what came back.  An output reads back the value this
           program published; an input reads what the last frame brought. */
        for (int i = 0; i < foundCt; i++)
        {
            const int32 v = (found[i].width == 1)
                ? (int32)(byte)var8[i].Read()
                : (int32)(uint16)var16[i].Read();

            printf("  SM%d (%s, %u byte(s)): 0x%04x\n",
                found[i].sm, found[i].isOutput ? "output" : "input",
                found[i].width, (unsigned)v);
        }

        cout << endl;

        CML::Thread::sleep(BECKHOFF_PDO_STEP_MS);

        count++;
        if (count == BECKHOFF_PDO_STEPS) return 0;
    }

    return 0;
}

/**************************************************/

/* Fatal: setup failed, there is nothing to run. */
static void showerr(const Error* err, const char* str)
{
    if (err)
    {
        printf("Error %s: %s\n", str, err->toString());
        exit(1);
    }
}
