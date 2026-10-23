/** \file

CML example: cyclic synchronous torque (DS402 mode 10), two axes

WHAT MODE 10 IS
---------------
The master streams one target TORQUE per axis per cycle and the drive
closes its CURRENT loop on it.

TWO THINGS MAKE THIS MODE UNLIKE THE OTHER TWO, RATHER THAN ONE MORE OF
THE SAME, AND BOTH CHANGE HOW YOUR APPLICATION IS SHAPED.

1.  NOTHING CML COMPUTES IS A TORQUE.  A trajectory is geometry: it yields
    a position and a velocity, and converting either into a torque needs
    the machine's inertia, friction and gravity, which this library has no
    model of.  So Linkage::SendTrajectory and StartMove REFUSE a mode-10
    linkage rather than streaming a number they made up, and the torques
    come from your application through Linkage::SendCstTorques.

2.  THERE IS NO CML STREAMING THREAD.  Your application computes the
    torques, so your application owns the clock: call SendCstTorques once
    per control cycle.  What CML owns is the part that is hard to get right
    from outside - the slew bound, the atomic multi-axis commit so both
    axes' torques land in the same frame, and the coordinated ramp to zero
    on HaltMove.

AND ZERO MEANS A THIRD THING
----------------------------
    mode 8   zero position error  = a no-op
    mode 9   zero velocity        = a stop
    mode 10  zero torque          = FREE-WHEEL

No force at all.  A horizontal axis coasts down on friction; a vertical
axis FALLS.  Zero is still the only value this library will invent on your
behalf, for mode 9's reason - any other number is a command nobody gave -
but being the honest choice does not make the axis safe, and nothing in a
commanded value can.

A STOPPED STREAM IS WORSE HERE THAN IN EITHER OF THE OTHERS.  Mode 8 holds
a position and mode 9 holds a velocity; mode 10 holds a FORCE, so the axis
keeps GAINING speed until the drive's watchdog disables it.


HOW THIS EXAMPLE RUNS
---------------------
It enables the axes at zero torque and streams a sine wave of torque,
CST_DEMO_TORQUE at its peak, until you press a key.  On a free motor the
rotor swings one way and then the other.  The key starts the coordinated
ramp to zero; the example then reports the cycle statistics and disables
the axes.

Run it only on an axis that can turn freely.  A torque is not a position:
how far the axis swings depends on its load and friction, not on anything
this example sets.


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

    make cst

which builds the CML library first if it needs building.  Or by hand, from
the top of the CML tree:

    g++ -O2 -g -I inc -I inc/can -I inc/ecat \
        -o cst examples/cst/cst.cpp libMotionLib.a -lpthread -lrt -ldl

On Windows, see readme.txt in this folder.

Run it from a terminal, from wherever your CML log should land; it writes
cst_cml.log.
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

/* The most torque this example will ever command.  2% of rated - enough to
   see the plumbing work and move a free motor slowly, not enough to be
   exciting.  RAISE THIS ONLY ON AN AXIS THAT CAN SPIN FREELY. */
#define CST_DEMO_TORQUE   20

/* The most torque the LIBRARY will pass, whatever the application asks
   for.  10% of rated here; set it from what your machine can take. */
static const int32 CST_MAX_TORQUE = 100;

/* How fast the torque may change, in the same thousandths-of-rated units
   PER SECOND.  1000/s means "the full rated torque in one second".

   This limit is load-bearing beyond the per-cycle check: without it
   HaltMove has no ramp to plan and drops the torque to zero in a single
   cycle.  Leave it set. */
static const int32 CST_TORQUE_SLEW = 500;

/* The torque wave's period.  Its steepest slope is 2 * pi *
   CST_DEMO_TORQUE per period - about 126/s here, well inside
   CST_TORQUE_SLEW. */
static const int  CST_WAVE_PERIOD_MS  = 1000;

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
 *  Helpers
 ***************************************************************************/

/* Axis 2 is driven the opposite way, so the two axes are visibly
   independent. */
static int AxisSign( int i ) { return ( i & 1 ) ? -1 : 1; }

static const double TWO_PI = 6.283185307179586;

/* The torque at time tMs: a sine wave, with the first half-wave at half
   height.  A wave that starts at full height pushes one way for a whole
   half-wave before it pulls back, so the velocity never changes sign and
   the axis creeps one way.  Half height for the first half-wave leaves the
   axis moving at the speed of a swing centred where it started, so from
   there it swings both ways about a fixed point.  Friction and load change
   the size of the swing, not that. */
static int16 CstWave( uint32 tMs )
{
   const double phase  = TWO_PI * (double)( tMs % CST_WAVE_PERIOD_MS )
                                / (double)CST_WAVE_PERIOD_MS;
   const double height = ( tMs < (uint32)( CST_WAVE_PERIOD_MS / 2 ) ) ? 0.5 : 1.0;

   return (int16)( CST_DEMO_TORQUE * height * sin( phase ) );
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
   cml.SetLogFile( "cst_cml.log" );
   cml.SetDebugLevel( LOG_EVERYTHING );

   printf( "CML example: cyclic synchronous torque (DS402 mode 10), two axes\n" );
   printf( "mode 10 (CST), two axes, " );
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
     3.  The Linkage, in CST mode.

         Linkage::Init does the whole cyclic bring-up: it remaps the PDOs
         for this mode, walks the nodes through Pre-Op, configures the sync
         managers, confirms the setpoints are seeded and brings everything
         back to Operational.  Every step of it is in the log.
    -----------------------------------------------------------------------*/
   Amp *ampPtr[AXIS_COUNT];
   for( int i=0; i<AXIS_COUNT; i++ ) ampPtr[i] = &amp[i];

   err = link.Init( AXIS_COUNT, ampPtr, LINKAGE_CYCLIC_CST );
   showerr( err, "initializing the linkage in CST mode" );

   err = link.SetMoveLimits( MAX_VEL, MAX_ACC, MAX_DEC, MAX_JRK );
   showerr( err, "setting the move limits" );

   /*-----------------------------------------------------------------------
     4.  The torque limits.  Both of them, before anything is enabled.

         Neither is refused if you leave it at zero - zero means "I have
         not told you", which this library has to accept, because a torque
         limit is a fact about your machine and inventing one would be
         worse than admitting it is unknown.  The bring-up warns instead,
         and the warning is in the log.
    -----------------------------------------------------------------------*/
   err = link.SetCstTorqueLimits( CST_MAX_TORQUE, CST_TORQUE_SLEW );
   showerr( err, "setting the CST torque limits" );

   printf( "Torque limits: max %d (%.1f%% of rated), slew %d/s\n",
           (int)CST_MAX_TORQUE, (double)CST_MAX_TORQUE / 10.0,
           (int)CST_TORQUE_SLEW );

   /* A per-axis limit, when the two axes are not the same size.  Zero on
      an axis means "use the linkage's", so a machine with matched motors
      never needs this. */
   /* err = link.SetCstAxisTorqueLimits( 1, 60, 300 ); */
   /* showerr( err, "setting axis 2's own torque limits" ); */

   /* The same two limits in NEWTON-METRES, if that is how your machine is
      specified.  Converts against each axis's own rated torque, which it
      reads from 0x6076 - so it needs the drives configured for the motors
      that are actually attached. */
   /* err = link.SetCstTorqueLimitsNm( 0.8, 4.0 ); */
   /* showerr( err, "setting the torque limits in Nm" ); */

   printf( "Effective slew in force: %d/s\n",
           (int)link.GetCstEffectiveSlew() );

   /*-----------------------------------------------------------------------
     5.  Enable.

         EnableAndHold warns on every call in this mode, because its name
         promises what mode 10 cannot do: it enables the axes at zero
         torque, which is free-wheel.
    -----------------------------------------------------------------------*/
   err = link.EnableAndHold();
   showerr( err, "enabling the linkage" );

   printf( "%s enabled at zero torque (free-wheeling).\n", AXES_TEXT );

   /*-----------------------------------------------------------------------
     6.  Stream until a key is pressed.

         This loop is your control loop.  CML is not pacing it: the cadence
         of these calls IS the cycle, and the slew bound is measured
         against the time that actually elapsed between them rather than
         against the nominal period - so a slow caller cannot step the
         torque faster than the limit it set.

         The wave is computed from the clock, not from a count of calls, so
         a late call does not stretch it.

         The key thread only sets a flag.  This loop owns the clock, so it
         is the loop that sees the flag and starts the halt.
    -----------------------------------------------------------------------*/
   StartKeyWatcher( 0 );

   printf( "\nStreaming a torque wave, peak %d, period %d ms, one call every "
           "%d ms.\nPress any key to halt.\n\n",
           (int)CST_DEMO_TORQUE, CST_WAVE_PERIOD_MS, CST_CALL_PERIOD_MS );

   int16 trq[AXIS_COUNT];

   const uint32 startMs   = Thread::getTimeMS();
   uint32       nextPrint = startMs;

   while( !keyPressed )
   {
      const uint32 nowMs = Thread::getTimeMS();
      const int16  want  = CstWave( nowMs - startMs );

      for( int i=0; i<AXIS_COUNT; i++ )
         trq[i] = (int16)( AxisSign( i ) * want );

      err = link.SendCstTorques( trq, AXIS_COUNT );
      showerr( err, "streaming a torque" );

      if( (int32)( nowMs - nextPrint ) >= 0 )
      {
         nextPrint += PRINT_PERIOD_MS;

         printf( "  %7.1f s:", (double)( nowMs - startMs ) / 1000.0 );
         for( int i=0; i<AXIS_COUNT; i++ )
            printf( "   axis %d torque %4d (measured %4d) velocity %8d", i + 1,
                    (int)trq[i], (int)amp[i].GetCstActualTorqueAtomic(),
                    (int)amp[i].GetCstActualVelAtomic() );
         printf( "\n" );
      }

      Thread::sleep( CST_CALL_PERIOD_MS );
   }

   /*-----------------------------------------------------------------------
     7.  The coordinated halt.

         HaltMove plans a ramp to zero at the slew limit and then OWNS the
         values: keep calling SendCstTorques exactly as before and the
         torques you hand over are ignored until the ramp reaches zero.
         Your loop does not change shape to halt - this one keeps
         computing the wave, and the ramp overrides it.
    -----------------------------------------------------------------------*/
   printf( "\nKey pressed - halting.  The ramp owns the torque from here...\n" );

   err = link.HaltMove();
   showerr( err, "halting" );

   const uint32 haltMs = Thread::getTimeMS();
   nextPrint = haltMs;

   while( !link.CstHaltDone() &&
          ( Thread::getTimeMS() - haltMs ) < (uint32)CST_HALT_TIMEOUT_MS )
   {
      const uint32 nowMs = Thread::getTimeMS();
      const int16  want  = CstWave( nowMs - startMs );

      for( int i=0; i<AXIS_COUNT; i++ )
         trq[i] = (int16)( AxisSign( i ) * want );

      err = link.SendCstTorques( trq, AXIS_COUNT );
      showerr( err, "streaming during the halt" );

      if( (int32)( nowMs - nextPrint ) >= 0 )
      {
         nextPrint += 20;

         printf( "  halting: asked for %4d, measured", (int)trq[0] );
         for( int i=0; i<AXIS_COUNT; i++ )
            printf( "   axis %d %4d", i + 1,
                    (int)amp[i].GetCstActualTorqueAtomic() );
         printf( "\n" );
      }

      Thread::sleep( CST_CALL_PERIOD_MS );
   }

   printf( "Halt %s.\n", link.CstHaltDone() ? "complete - torque at zero"
                                            : "DID NOT COMPLETE" );

   for( int i=0; i<AXIS_COUNT; i++ )
      printf( "  axis %d feedback seen: %s\n", i + 1,
              amp[i].CstFeedbackSeen() ? "yes" : "NO - no frame received" );


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

   printf( "\nDone.  The whole bring-up and run is in cst_cml.log.\n" );
   return 0;
}
