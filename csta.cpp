/** \file

CML example: cyclic torque with commutation angle (DS402 mode 11), two axes

WHAT MODE 11 IS
---------------
Mode 10 plus one field per cycle: object 0x60EA, the commutation angle.
Everything in the mode-10 example still applies - nothing CML computes is a
torque, there is no streaming thread, zero torque is free-wheel, and a
stopped stream keeps GAINING speed.

WHAT THE EXTRA FIELD CHANGES IS WHO COMMUTATES.  In mode 10 the drive
derives the electrical angle from its own feedback.  In mode 11 your
application supplies it, so your application is closing the commutation
loop and CML is carrying its answer.

The torque and the angle go out in the SAME FRAME, which is the whole
reason they are mapped into one PDO: a torque applied at the previous
cycle's commutation angle is a force in the wrong direction, and two PDOs
could be applied in different cycles.

THREE THINGS MAKE THE ANGLE NOT A SETPOINT
------------------------------------------
Every other quantity this library streams is bounded, zeroed on a stop, and
read back.  The angle is none of the three, and each is deliberate.

1.  IT IS NOT BOUNDED.  It wraps - see the block below.
2.  IT IS HELD, NOT ZEROED.  When a stream stops the torque falls to zero,
    because zero is a statement about a torque: no force.  There is no
    equivalent statement about an angle, and at zero torque no current
    flows so the angle selects nothing.  Writing a zero there would be the
    library inventing a rotor position.
3.  IT IS NOT READ BACK.  0x60EA is receive-only in the ESI.

Linkage::SendCstaTorques REFUSES a null angle array rather than filling it
in from the last one.  Holding the previous angle is right when nobody is
streaming; holding it while you ARE streaming a torque means commutating on
a stale rotor position while the rotor turns.


HOW THIS EXAMPLE RUNS
---------------------
It enables the axes and ramps the torque up to CSTA_SWEEP_TORQUE with the
commutation angle held still, so each rotor settles under the field.  Then
it sweeps the angle CSTA_SWEEP_ELEC_REVS electrical revolution one way and
back again, over and over, until you press a key.  The rotors follow the
field: this is mode 11 commutating open loop, with the application choosing
every angle.  The key starts the coordinated ramp of the torque to zero;
the example then shows the refusal of a send with no angles, reports the
cycle statistics and disables the axes.

Each axis must be free to move one electrical revolution - one pole pitch
on a linear motor, 1 / pole pairs of a turn on a rotary one - in either
direction from where it starts, plus up to half of one more while it
settles.  The sweep goes one way and back, axis 2's the opposite way to
axis 1's, and which way that is on the machine depends on the motor's
phasing.


WHICH TRANSPORT
---------------
USE_CAN is defined just below.  LEAVE IT IN for a CANopen network; COMMENT
IT OUT for EtherCAT.  Nothing else in this file changes between the two.

WHAT THIS EXAMPLE DRIVES
------------------------
A two-axis Linkage on one dual-axis drive.  How the second axis is reached
differs by transport, and it is not a style choice:

    EtherCAT   one node, two axes.  amp[0].Init( net, nodeID ) for axis 1,
               then amp[1].InitSubAxis( amp[0], 2 ) for axis 2.

    CANopen    two nodes.  A multi-axis CANopen drive presents each axis as
               its own node, so both axes use Amp::Init with their own node
               ID.  InitSubAxis is EtherCAT-only and says so in its own
               documentation.

A SINGLE-AXIS drive: set AXIS_COUNT to 1 below.  Only axis 1 is then
initialized and only axis 1's settings are used.

EMBEDDED LINUX / PREEMPT_RT
---------------------------
Everything a real-time deployment needs is in the SetUpRealTime() section
below, commented out with the reasoning.  Uncomment what applies to your
system.  Nothing in this example requires it - the defaults run on an
ordinary desktop kernel - but a 1 kHz cyclic stream on a loaded machine
will drop cycles without it, and the diagnostics at the end are how you
tell.

BUILD
-----
On Linux, from this folder:

    make csta

which builds the CML library first if it needs building.  Or by hand, from
the top of the CML tree:

    g++ -O2 -g -I inc -I inc/can -I inc/ecat \
        -o csta examples/csta/csta.cpp libMotionLib.a -lpthread -lrt -ldl

On Windows, see readme.txt in this folder.

Run it from a terminal, from wherever your CML log should land; it writes
csta_cml.log.
*/

/***************************************************************************
 *  TRANSPORT.  Leave this defined for CANopen; comment it out for EtherCAT.
 ***************************************************************************/
#define USE_CAN

#include <cstdio>
#include <cstdlib>

#include "CML.h"

#if defined( USE_CAN )
#  include "can/can_copley.h"        /* or can_kvaser.h, can_peak.h, ... */
#else
#  if defined( WIN32 )
#    include "ecat/ecat_winudp.h"
#  else
#    include "ecat/ecat_linux.h"
#  endif
#endif

#include <atomic>
#include <thread>
#include <cmath>

/* The key watcher's platform headers, after CML's own so that the Windows
   socket headers the EtherCAT transport needs come first. */
#if defined( _WIN32 )
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  ifndef NOMINMAX
#    define NOMINMAX
#  endif
#  include <windows.h>
#  include <conio.h>
#else
#  include <termios.h>
#  include <unistd.h>
#endif

CML_NAMESPACE_USE();

/***************************************************************************
 *  YOUR MACHINE.  These are the only lines most people need to change.
 ***************************************************************************/

/* How many axes the drive has: 2 for a dual-axis drive, 1 for a
   single-axis drive. */
#define AXIS_COUNT 2

#if ( AXIS_COUNT != 1 ) && ( AXIS_COUNT != 2 )
#  error "AXIS_COUNT must be 1 or 2"
#endif

#if defined( USE_CAN )

/* A dual-axis CANopen drive is two nodes.  Set both. */
static const int16 AXIS_1_NODE_ID = 1;
static const int16 AXIS_2_NODE_ID = 2;

/* CAN port and bit rate.  The bit rate is in bits per second. */
static const char *CAN_PORT = "CAN0";
static const int32 CAN_BPS  = 1000000;

/* SYNC period, in MICROSECONDS.  This is the cyclic rate on CANopen: every
   mapped PDO is applied and sampled on the SYNC.  10000us = 100 Hz is a
   safe starting point on 1 Mbit with two axes; come down from there once
   the bus load is known. */
static const uint32 SYNC_PERIOD_US = 10000;

#else

/* A dual-axis EtherCAT drive is ONE node with two axes. */
static const int16 DRIVE_NODE_ID = -1;        /* -1 = first node found    */

/* The NIC.  On Windows this is the IP the UDP transport binds to; on Linux
   it is the interface name.  Declared per platform so an unused one cannot
   warn - an example that compiles with warnings teaches the wrong habit. */
#  if defined( WIN32 )
static const char *ECAT_WIN_IP    = "192.168.0.33";
#  else
static const char *ECAT_INTERFACE = "eth0";
#  endif

/* Cycle period in MILLISECONDS.  5 ms holds on a desktop Windows timer;
   a real-time system can go much lower. */
static const int ECAT_CYCLE_MS = 5;

#endif

/* Move limits, in user units.  Set these from your own machine before you
   run anything: they bound every setpoint this library sends. */
static const uunit MAX_VEL = 100000;
static const uunit MAX_ACC = 500000;
static const uunit MAX_DEC = 500000;
static const uunit MAX_JRK = 5000000;

/***************************************************************************
 *  TORQUE LIMITS.  Read this block before you run anything.
 *
 *  0x6071 is in THOUSANDTHS OF RATED TORQUE: 1000 is rated torque, 100 is
 *  10%, 20 is 2%.  The unit is already normalised per axis, so the same
 *  number means the same fraction of rating on a big motor and a small one.
 ***************************************************************************/

/* The torque this example commands while it sweeps: 20% of rated.  The
   field drags the rotor along, so this sets how firmly the rotor follows,
   not how fast it moves - the sweep sets the speed and the travel.  The 2%
   that mode 10's example uses is usually lost in friction here.  RUN THIS
   ONLY ON AN AXIS THAT CAN MOVE FREELY. */
#define CSTA_SWEEP_TORQUE   200

/* The most torque the LIBRARY will pass, whatever the application asks
   for.  25% of rated here, just above what this example asks for; set it
   from what your machine can take. */
static const int32 CST_MAX_TORQUE = 250;

/* How fast the torque may change, in the same thousandths-of-rated units
   PER SECOND.  1000/s means "the full rated torque in one second".

   This limit is load-bearing beyond the per-cycle check: without it
   HaltMove has no ramp to plan and drops the torque to zero in a single
   cycle.  Leave it set. */
static const int32 CST_TORQUE_SLEW = 500;

/* How long the torque takes to ramp in, with the angle held: 200/s, well
   inside CST_TORQUE_SLEW. */
static const int  CSTA_RAMP_MS        = 1000;

/* How long the halt ramp may take before the example reports that it did
   not complete, and how often the loop prints. */
static const int  CST_HALT_TIMEOUT_MS = 3000;
static const int  PRINT_PERIOD_MS     = 500;

/* How often the application calls; in a torque mode the application owns
   the clock - see below.  One call per cyclic update on EtherCAT, so each
   call is one setpoint the drive receives.  On CANopen the SYNC sets the
   pace and the calls refresh the setpoint between SYNCs. */
#if defined( USE_CAN )
static const int  CST_CALL_PERIOD_MS = 2;
#else
static const int  CST_CALL_PERIOD_MS = ECAT_CYCLE_MS;
#endif

/***************************************************************************
 *  THE COMMUTATION ANGLE.
 *
 *  0x60EA is an UNSIGNED WRAPPING electrical angle: 0 to 0xFFFF covers one
 *  full electrical revolution, every value in it is legal, and the step
 *  from 0xFFFF to 0x0000 is ONE count rather than 65535.
 *
 *  CML does not bound it, and that is a decision rather than an omission.
 *  A bound expressed in counts cannot tell a legal wrap from a wild jump,
 *  so it would reject exactly the commutation an axis turning through zero
 *  needs - once per revolution, forever - and catch nothing else.  The
 *  thing that can judge a commutation angle is the drive, because it has
 *  the current loop.
 *
 *  SO THE ANGLE IS ENTIRELY YOURS.  This example sweeps it open loop: the
 *  angle comes from the clock, not from the rotor, and the rotor follows
 *  the field the way a stepper follows its phases.  That needs no motor
 *  numbers, and it shows the field reaching the drive in the same frame
 *  as its torque.  The live derivation is below, commented, with the two
 *  constants only you can supply.
 ***************************************************************************/

/* Where the field points while the torque ramps in.  Any value will do: the
   rotor settles under it. */
static const uint16 CSTA_START_ANGLE = 0x0000;

/* The sweep: CSTA_SWEEP_ELEC_REVS electrical revolution one way and back
   again, once every CSTA_SWEEP_PERIOD_MS.  The fastest the field turns is
   pi * CSTA_SWEEP_ELEC_REVS / period - about 0.8 electrical revolutions a
   second here, slow enough for a rotor to keep up from a standstill. */
static const double CSTA_SWEEP_ELEC_REVS = 1.0;
static const int    CSTA_SWEEP_PERIOD_MS = 4000;

/* ---- the live derivation, which needs your motor's numbers -------------

   The electrical angle is the rotor's position within ONE electrical
   revolution, scaled to 16 bits.  One electrical revolution is one
   mechanical revolution divided by the number of POLE PAIRS.

       static const int32 COUNTS_PER_MECH_REV = 65536;   // your encoder
       static const int32 POLE_PAIRS          = 4;       // your motor

       static uint16 ElectricalAngle( int32 posCounts )
       {
          const int32 countsPerElecRev = COUNTS_PER_MECH_REV / POLE_PAIRS;

          // Wrap into [0, countsPerElecRev) with a sign-safe modulo: a
          // negative position must not produce a negative angle.
          int32 within = posCounts % countsPerElecRev;
          if( within < 0 ) within += countsPerElecRev;

          // Scale to the full 16-bit range.  64-bit intermediate, because
          // within * 65536 overflows int32 for any realistic encoder.
          return (uint16)( ( (int64)within * 65536 ) / countsPerElecRev );
       }

   and then, for each axis in the streaming loop below:

       angle[i] = ElectricalAngle( amp[i].GetCstaActualPosAtomic() );

   A WRONG CONSTANT HERE IS A FORCE IN THE WRONG DIRECTION.  Expect to
   verify it on a free axis at low torque before trusting it:

     - a torque RIPPLE at the electrical frequency means the angle has a
       fixed offset;
     - a torque that FALLS OFF with speed means the angle is lagging;
     - an axis that accelerates with every commanded torque inside its
       limit is commutating wrong, and that is the one failure mode this
       mode has and mode 10 does not.

   There is no readback to check it against.  0x60EA is PdoMapping R in the
   ESI - the drive receives it and never transmits it - so the nearest
   thing is Amp::GetCstaActualPosAtomic, which is why mode 11's CANopen
   feedback carries position where mode 10's carries velocity.
  ----------------------------------------------------------------------- */


/***************************************************************************
 *  Helpers
 ***************************************************************************/

/* Axis 2 is driven the opposite way, so the two axes are visibly
   independent. */
static int AxisSign( int i ) { return ( i & 1 ) ? -1 : 1; }

static const double TWO_PI = 6.283185307179586;

/* The torque at time tMs: a ramp up to CSTA_SWEEP_TORQUE, then held. */
static int16 CstaTorque( uint32 tMs )
{
   if( tMs >= (uint32)CSTA_RAMP_MS ) return CSTA_SWEEP_TORQUE;

   return (int16)( (int32)CSTA_SWEEP_TORQUE * (int32)tMs / CSTA_RAMP_MS );
}

/* The electrical angle at time tMs, for an axis whose field turns the way
   sign says.  Held at CSTA_START_ANGLE while the torque ramps in, then a
   cosine sweep out and back, so the field starts and reverses at zero
   speed.  The angle wraps, and unsigned arithmetic does that for free. */
static uint16 CstaAngle( uint32 tMs, int sign )
{
   if( tMs <= (uint32)CSTA_RAMP_MS ) return CSTA_START_ANGLE;

   const double phase = TWO_PI
                      * (double)( ( tMs - CSTA_RAMP_MS ) % CSTA_SWEEP_PERIOD_MS )
                      / (double)CSTA_SWEEP_PERIOD_MS;
   const double revs  = CSTA_SWEEP_ELEC_REVS * ( 1.0 - cos( phase ) ) / 2.0;
   const int32  delta = sign * (int32)( revs * 65536.0 );

   return (uint16)( CSTA_START_ANGLE + (uint32)delta );
}

#define AXES_TEXT ( ( AXIS_COUNT > 1 ) ? "Both axes" : "The axis" )

/* Fatal: there is nothing useful to do after a setup failure, and carrying
   on with a half-configured linkage is how a test run damages hardware. */
static void showerr( const Error *err, const char *what )
{
   if( err )
   {
      printf( "ERROR %s: %s\n", what, err->toString() );
      exit( 1 );
   }
}

/* Print the cycle statistics.  This is the part worth keeping in your own
   application: it is how you find out whether the stream actually held its
   period, and it costs nothing to read. */
static void showCycleStats( Linkage &link )
{
   CspDiag d;
   link.GetCspDiagnostics( d );

   printf( "\n--- cycle statistics -------------------------------------\n" );
   printf( "  cycles           : %u\n",        (unsigned)d.period.cycles );
   printf( "  nominal period   : %u us\n",     (unsigned)d.period.nominalUs );
   printf( "  measured min/mean/max : %u / %u / %u us\n",
           (unsigned)d.period.minUs, (unsigned)d.period.meanUs,
           (unsigned)d.period.maxUs );
   printf( "  p99 / p99.9      : %u / %u us\n",
           (unsigned)d.period.p99Us, (unsigned)d.period.p999Us );
   printf( "  late / very late : %u / %u\n",
           (unsigned)d.period.lateCycles, (unsigned)d.period.veryLateCycles );
   printf( "  lost cycles      : %u\n",        (unsigned)d.lostCycles );
   printf( "  setpoints refused: %u\n",        (unsigned)d.rejected );

   if( d.haveSlack )
      printf( "  worst slack      : %u us\n",  (unsigned)d.worstSlackUs );
   else
      printf( "  worst slack      : none - no cycle had any time in hand.\n"
              "                     The period itself is in question, not\n"
              "                     the sleep's accuracy.\n" );

   printf( "----------------------------------------------------------\n" );
}

/***************************************************************************
 *  STOPPING ON A KEY
 *
 *  The motion runs until a key is pressed.  A plain thread waits for the
 *  key, so nothing in the motion loop polls the keyboard.  It is a
 *  std::thread rather than a CML Thread on purpose: CML threads run at a
 *  real-time priority on POSIX, and a thread that waits on a person has no
 *  business there.
 *
 *  Ctrl+C counts as a key.  Killing the program would end the stream with
 *  no coordinated halt, so it halts instead.
 *
 *  On Linux the terminal is switched to single-key mode (no Enter, no
 *  echo) and switched back on the way out.  With no terminal on stdin, end
 *  of input counts as the key, so the run halts almost at once.
 ***************************************************************************/
static std::atomic<bool> keyPressed( false );
static void ( *onKey )( void ) = 0;

static void KeyPressed( void )
{
   /* The first key only: on Windows a Ctrl+C can arrive alongside it. */
   if( keyPressed.exchange( true ) ) return;
   if( onKey ) onKey();
}

#if defined( _WIN32 )

static BOOL WINAPI CtrlHandler( DWORD type )
{
   if( ( type != CTRL_C_EVENT ) && ( type != CTRL_BREAK_EVENT ) )
      return FALSE;

   KeyPressed();
   return TRUE;
}

static void WatchKeys( void )
{
   _getch();
   KeyPressed();
}

static void StartKeyWatcher( void ( *hook )( void ) )
{
   onKey = hook;
   SetConsoleCtrlHandler( CtrlHandler, TRUE );
   std::thread( WatchKeys ).detach();
}

#else

static struct termios    savedTerm;
static std::atomic<bool> termChanged( false );

static void RestoreTerminal( void )
{
   if( termChanged.exchange( false ) )
      tcsetattr( STDIN_FILENO, TCSANOW, &savedTerm );
}

static void WatchKeys( void )
{
   /* A key, end of input or an error: each one ends the run. */
   unsigned char c;
   ssize_t got = read( STDIN_FILENO, &c, 1 );
   (void)got;

   /* The rest of a multi-byte key, so it does not reach the shell. */
   tcflush( STDIN_FILENO, TCIFLUSH );
   RestoreTerminal();

   KeyPressed();
}

static void StartKeyWatcher( void ( *hook )( void ) )
{
   onKey = hook;

   struct termios t;
   if( isatty( STDIN_FILENO ) && ( tcgetattr( STDIN_FILENO, &t ) == 0 ) )
   {
      savedTerm = t;

      /* ISIG off is what makes Ctrl+C a key rather than a signal. */
      t.c_lflag &= ~( ICANON | ECHO | ISIG );
      t.c_cc[VMIN]  = 1;
      t.c_cc[VTIME] = 0;

      if( tcsetattr( STDIN_FILENO, TCSANOW, &t ) == 0 )
      {
         termChanged = true;
         atexit( RestoreTerminal );
      }
   }
   else
      printf( "stdin is not a terminal: end of input will halt the run.\n" );

   std::thread( WatchKeys ).detach();
}

#endif

/***************************************************************************
 *  REAL-TIME SETUP FOR EMBEDDED LINUX
 *
 *  Every line in this function is commented out.  None of it is needed to
 *  make the example run; all of it matters on a real machine at a real
 *  cycle rate.  Uncomment what applies and leave the rest.
 *
 *  The numbers that tell you whether it worked are the ones
 *  showCycleStats prints.
 ***************************************************************************/
#if defined( USE_CAN )
static void SetUpRealTime( CanOpenSettings &settings, Linkage &link )
#else
static void SetUpRealTime( EtherCatSettings &settings, Linkage &link )
#endif
{
   /* Silence the unused-parameter warnings while everything below is
      commented out.  Delete these two lines as you uncomment. */
   (void)settings;
   (void)link;

   /*=======================================================================
     1.  KERNEL.  Not code - but nothing below helps without it.

         A PREEMPT_RT kernel, and a core reserved for the cyclic work:

             isolcpus=4,5  nohz_full=4,5  rcu_nocbs=4,5

         isolcpus keeps the general scheduler off those cores, nohz_full
         stops the periodic timer tick on them, and rcu_nocbs moves RCU
         callback processing away.  A core that still takes a 250us tick
         cannot hold a 1 ms period reliably no matter how the threads are
         configured.
    =======================================================================*/

   /*=======================================================================
     2.  THREAD PRIORITY.  CML's threads run SCHED_FIFO when the process
         has the privilege for it (CAP_SYS_NICE, or run as root).  These
         are relative priorities inside CML's own scheme, not raw kernel
         numbers.

         Keep them BELOW any hard interrupt-thread priority on your system,
         and keep the cycle thread at or above the read thread: the cycle
         thread has a deadline, the read thread does not.
    =======================================================================*/
   /* settings.cycleThreadPriority = 9; */      /* the default */
   /* settings.readThreadPriority  = 9; */

   /*=======================================================================
     3.  AFFINITY.  Pin the cyclic work to the isolated core.

         This is the single most effective change on a loaded machine.  -1,
         the default, leaves placement to the scheduler, which will migrate
         the thread off a warm cache at the worst possible moment.
    =======================================================================*/
#if !defined( USE_CAN )
   /* settings.cycleThreadAffinity = 4; */      /* an isolated core */
   /* settings.readThreadAffinity  = -1; */     /* fine on the general pool */
#endif

   /*  And the setpoint stream's own thread, which is a Linkage thread
       rather than a network thread.  Takes effect at the start of the next
       move, because that is when the thread is created.

       Pin it to a DIFFERENT core from the cycle thread unless that core is
       genuinely idle: the two run at the same rate and will fight. */
   /* link.SetCspThreadAffinity( 5 ); */

   /*=======================================================================
     4.  BUSY-WAIT THE LAST FEW MICROSECONDS.  EtherCAT only.

         A timed sleep wakes up late by whatever the scheduler's granularity
         is, and on a general kernel that can be a large fraction of a 1 ms
         period.  Waking deliberately early and spinning the remainder takes
         the wake-up out of the timing path entirely.

         It costs CPU: the thread burns its core for the whole spin.  Do NOT
         set this without pinning the cycle thread first (step 3) - on a
         shared core a busy-wait competes with the work it is waiting for.

             > 0   sleep until this many us before the deadline, then spin.
                   A few tens of us is usually enough.
             < 0   spin the ENTIRE interval; the thread never sleeps.  Only
                   on a dedicated, kernel-isolated core.
    =======================================================================*/
#if !defined( USE_CAN )
   /* settings.cycleThreadSpinUs = 50; */
#endif

   /*=======================================================================
     5.  LOCK MEMORY.  A page fault in the cyclic path costs milliseconds.

             #include <sys/mman.h>
             mlockall( MCL_CURRENT | MCL_FUTURE );

         Do this once, early, before the network is opened.  Also consider
         pre-faulting your own stacks and turning off malloc trimming
         (mallopt( M_TRIM_THRESHOLD, -1 ), mallopt( M_MMAP_MAX, 0 )).
    =======================================================================*/

   /*=======================================================================
     6.  WHAT NOT TO DO IN THE CYCLIC PATH

         No file I/O, no logging, no allocation, no mutex shared with a
         non-real-time thread.  CML's own cyclic path takes none of those.
         If your setpoint producer does, it will show up as late cycles in
         the statistics rather than as an error.
    =======================================================================*/
}

/***************************************************************************
 *  main
 ***************************************************************************/
int main( void )
{
   /* LOG_EVERYTHING while you are bringing a machine up.  It is the single
      most useful thing you can send to Copley support, and it is what the
      bring-up sequence writes its findings to. */
   cml.SetLogFile( "csta_cml.log" );
   cml.SetDebugLevel( LOG_EVERYTHING );

   printf( "CML example: cyclic torque with commutation angle (DS402 mode 11), two axes\n" );
   printf( "mode 11 (CSTA), two axes, " );
#if defined( USE_CAN )
   printf( "CANopen\n\n" );
#else
   printf( "EtherCAT\n\n" );
#endif

   /*-----------------------------------------------------------------------
     0.  The warning this mode earns.
    -----------------------------------------------------------------------*/
   printf( "  ZERO TORQUE IS FREE-WHEEL, NOT A HOLD.\n" );
   printf( "  Nothing in this mode resists gravity or an external push.\n" );
   printf( "  A vertical axis needs its brake engaged before you run this.\n" );
   printf( "  A stream that stops leaves the last torque applied, and a\n" );
   printf( "  torque command has no endpoint - an unopposed axis keeps\n" );
   printf( "  accelerating until something stops it.\n\n" );

   printf( "  AND IN THIS MODE YOUR APPLICATION COMMUTATES.\n" );
   printf( "  0x60EA is not bounded by CML and cannot be read back.  An\n" );
   printf( "  axis that runs away with every torque inside its limit is\n" );
   printf( "  commutating wrong - that is this mode's own failure.\n\n" );


   /*-----------------------------------------------------------------------
     1.  Open the network.
    -----------------------------------------------------------------------*/
#if defined( USE_CAN )

   CopleyCAN         hw( CAN_PORT );
   CanOpen           net;
   CanOpenSettings   netSettings;

   hw.SetBaud( CAN_BPS );

   Linkage link;                      /* declared early for SetUpRealTime */

   SetUpRealTime( netSettings, link );

   const Error *err = net.Open( hw, netSettings );
   showerr( err, "opening the CANopen network" );

#else

#  if defined( WIN32 )
   WinUdpEcatHardware hw( ECAT_WIN_IP );
#  else
   LinuxEcatHardware  hw( ECAT_INTERFACE );
#  endif

   EtherCAT          net;
   EtherCatSettings  netSettings;

   netSettings.cyclePeriod = ECAT_CYCLE_MS;

   Linkage link;                      /* declared early for SetUpRealTime */

   SetUpRealTime( netSettings, link );

   const Error *err = net.Open( hw, netSettings );
   showerr( err, "opening the EtherCAT network" );

#endif

   /*-----------------------------------------------------------------------
     2.  The two axes.
    -----------------------------------------------------------------------*/
   Amp amp[AXIS_COUNT];

#if defined( USE_CAN )

   /* On CANopen the SYNC period is the cyclic rate, and it is an AMP
      setting rather than a network one.  Both axes must agree. */
   AmpSettings ampSettings;
   ampSettings.synchPeriod = SYNC_PERIOD_US;

   err = amp[0].Init( net, AXIS_1_NODE_ID, ampSettings );
   showerr( err, "initializing axis 1" );

#  if AXIS_COUNT > 1
   err = amp[1].Init( net, AXIS_2_NODE_ID, ampSettings );
   showerr( err, "initializing axis 2" );
#  endif

#else

   err = amp[0].Init( net, DRIVE_NODE_ID );
   showerr( err, "initializing axis 1" );

   /* Axis 2 of the SAME drive.  Not a second Init - a sub-axis of the
      first, which is how a multi-axis EtherCAT drive is addressed. */
#  if AXIS_COUNT > 1
   err = amp[1].InitSubAxis( amp[0], 2 );
   showerr( err, "initializing axis 2" );
#  endif

#endif

   /*-----------------------------------------------------------------------
     3.  The Linkage, in CSTA mode.

         Linkage::Init does the whole cyclic bring-up: it remaps the PDOs
         for this mode, walks the nodes through Pre-Op, configures the sync
         managers, confirms the setpoints are seeded and brings everything
         back to Operational.  Every step of it is in the log.
    -----------------------------------------------------------------------*/
   Amp *ampPtr[AXIS_COUNT];
   for( int i=0; i<AXIS_COUNT; i++ ) ampPtr[i] = &amp[i];

   err = link.Init( AXIS_COUNT, ampPtr, LINKAGE_CYCLIC_CSTA );
   showerr( err, "initializing the linkage in CSTA mode" );

   err = link.SetMoveLimits( MAX_VEL, MAX_ACC, MAX_DEC, MAX_JRK );
   showerr( err, "setting the move limits" );

   /*-----------------------------------------------------------------------
     4.  The torque limits.  Mode 11's torques are bounded by exactly the
         same arithmetic as mode 10's - same call, same units, same slew.
         The ANGLE is bounded by nothing; see the block at the top.
    -----------------------------------------------------------------------*/
   err = link.SetCstTorqueLimits( CST_MAX_TORQUE, CST_TORQUE_SLEW );
   showerr( err, "setting the torque limits" );

   printf( "Torque limits: max %d (%.1f%% of rated), slew %d/s\n",
           (int)CST_MAX_TORQUE, (double)CST_MAX_TORQUE / 10.0,
           (int)CST_TORQUE_SLEW );
   printf( "Commutation angle: swept open loop from 0x%04X (see the file "
           "header).\n", (unsigned)CSTA_START_ANGLE );

   /*-----------------------------------------------------------------------
     5.  Enable.  Same free-wheel warning as mode 10, and the angle does
         not change it: at zero torque no current flows whatever angle is
         in the field, so nothing is holding there either.
    -----------------------------------------------------------------------*/
   err = link.EnableAndHold();
   showerr( err, "enabling the linkage" );

   printf( "%s enabled at zero torque (free-wheeling).\n", AXES_TEXT );

   /*-----------------------------------------------------------------------
     6.  Stream the torque AND the angle until a key is pressed.

         The only difference from mode 10 is the second array.  Both
         entry points share one implementation inside CML, so the clock
         measurement, the slew bound, the atomic multi-axis commit and the
         halt ramp are the same code.

         The torque ramps in with the angle held, then the angle sweeps;
         see CstaTorque and CstaAngle.  Both come from the clock, not from
         a count of calls, so a late call does not stretch them.

         The key thread only sets a flag.  This loop owns the clock, so it
         is the loop that sees the flag and starts the halt.
    -----------------------------------------------------------------------*/
   StartKeyWatcher( 0 );

   printf( "\nRamping the torque in over %d ms, then sweeping the angle %.1f "
           "electrical rev out and back every %d ms,\none call every %d ms.  "
           "Press any key to halt.\n\n",
           CSTA_RAMP_MS, CSTA_SWEEP_ELEC_REVS, CSTA_SWEEP_PERIOD_MS,
           CST_CALL_PERIOD_MS );

   int16  trq[AXIS_COUNT];
   uint16 angle[AXIS_COUNT];

   const uint32 startMs   = Thread::getTimeMS();
   uint32       nextPrint = startMs;

   while( !keyPressed )
   {
      const uint32 nowMs = Thread::getTimeMS();
      const uint32 tMs   = nowMs - startMs;

      /* The open-loop angle.  Replace it with the live derivation from the
         file header once you have the pole count:

             angle[i] = ElectricalAngle( amp[i].GetCstaActualPosAtomic() );   */
      for( int i=0; i<AXIS_COUNT; i++ )
      {
         trq[i]   = CstaTorque( tMs );
         angle[i] = CstaAngle( tMs, AxisSign( i ) );
      }

      err = link.SendCstaTorques( trq, angle, AXIS_COUNT );
      showerr( err, "streaming a torque and angle" );

      if( (int32)( nowMs - nextPrint ) >= 0 )
      {
         nextPrint += PRINT_PERIOD_MS;

         printf( "  %7.1f s:", (double)tMs / 1000.0 );
         for( int i=0; i<AXIS_COUNT; i++ )
            printf( "   axis %d angle 0x%04X torque %4d (measured %4d) pos %d",
                    i + 1, (unsigned)angle[i], (int)trq[i],
                    (int)amp[i].GetCstaActualTorqueAtomic(),
                    (int)amp[i].GetCstaActualPosAtomic() );
         printf( "\n" );
      }

      Thread::sleep( CST_CALL_PERIOD_MS );
   }

   /*-----------------------------------------------------------------------
     7.  The halt.

         The ramp owns the TORQUE and leaves the ANGLE alone: keep sending
         the angle your commutation says is correct.  Here the sweep stops
         where it is, so the rotor comes to rest under a still field while
         the ramp takes the torque away.  The torque asked for stays at
         the full sweep value, to show that the ramp overrides it.
    -----------------------------------------------------------------------*/
   printf( "\nKey pressed - halting.  The ramp owns the torque, you keep the angle...\n" );

   err = link.HaltMove();
   showerr( err, "halting" );

   for( int i=0; i<AXIS_COUNT; i++ )
      trq[i] = CSTA_SWEEP_TORQUE;

   const uint32 haltMs = Thread::getTimeMS();
   nextPrint = haltMs;

   while( !link.CstHaltDone() &&
          ( Thread::getTimeMS() - haltMs ) < (uint32)CST_HALT_TIMEOUT_MS )
   {
      err = link.SendCstaTorques( trq, angle, AXIS_COUNT );
      showerr( err, "streaming during the halt" );

      const uint32 nowMs = Thread::getTimeMS();
      if( (int32)( nowMs - nextPrint ) >= 0 )
      {
         nextPrint += 50;

         printf( "  halting: asked for %4d, measured", (int)trq[0] );
         for( int i=0; i<AXIS_COUNT; i++ )
            printf( "   axis %d %4d", i + 1,
                    (int)amp[i].GetCstaActualTorqueAtomic() );
         printf( "\n" );
      }

      Thread::sleep( CST_CALL_PERIOD_MS );
   }

   printf( "Halt %s.\n", link.CstHaltDone() ? "complete - torque at zero"
                                            : "DID NOT COMPLETE" );

   /*-----------------------------------------------------------------------
     8.  And the refusal, executed, because it is the one error in this
         mode an application will actually hit.
    -----------------------------------------------------------------------*/
   const Error *nullErr = link.SendCstaTorques( trq, 0, AXIS_COUNT );

   printf( "\nA mode-11 send with no angles is refused: %s\n",
           nullErr ? nullErr->toString() : "ACCEPTED - that is a defect" );

   for( int i=0; i<AXIS_COUNT; i++ )
      printf( "  axis %d feedback seen: %s\n", i + 1,
              amp[i].CstaFeedbackSeen() ? "yes" : "NO - no frame received" );


   /*-----------------------------------------------------------------------
     Done.  Disable both axes and report.
    -----------------------------------------------------------------------*/
   showCycleStats( link );

   for( int i=0; i<AXIS_COUNT; i++ )
   {
      const Error *dErr = amp[i].Disable();

      if( dErr ) printf( "WARNING: axis %d would not disable: %s\n",
                         i + 1, dErr->toString() );
   }

   printf( "\nDone.  The whole bring-up and run is in csta_cml.log.\n" );
   return 0;
}
