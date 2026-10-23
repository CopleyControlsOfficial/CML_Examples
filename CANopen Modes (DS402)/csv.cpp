/** \file

CML example: cyclic synchronous velocity (DS402 mode 9), two axes

WHAT MODE 9 IS
--------------
The master streams one target VELOCITY per axis per cycle and the drive
closes its VELOCITY loop on it.  There is no position loop in the drive at
all.

CML still plans the trajectory, so the application side looks almost
exactly like mode 8: hand the linkage a move and its streaming thread sends
the velocity the trajectory is already computing.  What changes is what
happens when something goes wrong.

THE ONE THING TO UNDERSTAND BEFORE RUNNING THIS
-----------------------------------------------
A STREAM THAT STOPS IS A RUNAWAY, NOT A STALL.

A mode-8 drive whose stream stops keeps being told the same position and
stays put.  A mode-9 drive whose stream stops keeps being told the same
VELOCITY and keeps going, until its own watchdog disables it.  So:

  - "stop streaming" is not a way to stop an axis.  HaltMove is: it ramps
    to zero and then keeps sending zero.
  - zero velocity is a STOP, which is why it is the only value this library
    will ever invent on your behalf.
  - Linkage::EnableAndHold commands zero, which in this mode really does
    hold the axis still.


HOW THIS EXAMPLE RUNS
---------------------
It enables the axes and moves them back and forth, MOVE_DISTANCE each way,
until you press a key.  The key halts the move wherever it has got to, with
Linkage::HaltMove; the example then reports the cycle statistics and
disables the axes.

Nothing holds a position in this mode, so a stroke does not end exactly
where the last one started: the end points wander a little from stroke to
stroke.  That is the mode, not a fault.  Each axis must be free to travel
MOVE_DISTANCE from where it starts, axis 1 in the positive direction and
axis 2 in the negative, with some margin for that.


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

    make csv

which builds the CML library first if it needs building.  Or by hand, from
the top of the CML tree:

    g++ -O2 -g -I inc -I inc/can -I inc/ecat \
        -o csv examples/csv/csv.cpp libMotionLib.a -lpthread -lrt -ldl

On Windows, see readme.txt in this folder.

Run it from a terminal, from wherever your CML log should land; it writes
csv_cml.log.
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
#include <mutex>

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

/* How far each stroke moves, in user units.  Relative, so the example does
   not need a home and does not care where the axes started. */
static const uunit MOVE_DISTANCE = 20000;

/* The rest at each end of a stroke, and the longest one stroke may take
   before the example gives up on it. */
static const int STROKE_DWELL_MS   = 250;
static const int STROKE_TIMEOUT_MS = 20000;

/* When to sample the velocity, after a stroke starts. */
static const int MID_STROKE_MS     = 200;


/***************************************************************************
 *  Helpers
 ***************************************************************************/

/* Axis 2 is driven the opposite way, so the two axes are visibly
   independent. */
static int AxisSign( int i ) { return ( i & 1 ) ? -1 : 1; }

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

/* The halt the key triggers.  The main loop starts every stroke holding
   moveMtx and this takes it too, so a key cannot land between the loop's
   last look at keyPressed and the stroke starting: either the stroke never
   starts, or it starts and this halts it. */
static std::mutex  moveMtx;
static Linkage    *haltLink = 0;

static void HaltOnKey( void )
{
   std::lock_guard<std::mutex> lock( moveMtx );

   printf( "\nKey pressed - halting.\n" );

   const Error *err = haltLink->HaltMove();
   if( err ) printf( "WARNING: HaltMove failed: %s\n", err->toString() );
}

/* GetCspDiagnostics describes one move, and every stroke is a move of its
   own, so the strokes are added up here for the whole run. */
static uint32 runCycles = 0, runMaxUs = 0, runLate = 0, runVeryLate = 0,
              runLost = 0, runRefused = 0;

/* Adds the move that just ended, and returns its worst period. */
static uint32 AddStroke( Linkage &link )
{
   CspDiag d;
   link.GetCspDiagnostics( d );

   runCycles   += d.period.cycles;
   runLate     += d.period.lateCycles;
   runVeryLate += d.period.veryLateCycles;
   runLost     += d.lostCycles;
   runRefused  += d.rejected;
   if( d.period.maxUs > runMaxUs ) runMaxUs = d.period.maxUs;

   return d.period.maxUs;
}

static void showRunTotals( void )
{
   printf( "\n--- the whole run ----------------------------------------\n" );
   printf( "  cycles           : %u\n",    (unsigned)runCycles );
   printf( "  worst period     : %u us\n", (unsigned)runMaxUs );
   printf( "  late / very late : %u / %u\n", (unsigned)runLate, (unsigned)runVeryLate );
   printf( "  lost cycles      : %u\n",    (unsigned)runLost );
   printf( "  setpoints refused: %u\n",    (unsigned)runRefused );
   printf( "----------------------------------------------------------\n" );
   printf( "The statistics below are the last stroke's, the one the key halted.\n" );
}

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
   cml.SetLogFile( "csv_cml.log" );
   cml.SetDebugLevel( LOG_EVERYTHING );

   printf( "CML example: cyclic synchronous velocity (DS402 mode 9), two axes\n" );
   printf( "mode 9 (CSV), two axes, " );
#if defined( USE_CAN )
   printf( "CANopen\n\n" );
#else
   printf( "EtherCAT\n\n" );
#endif


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
     3.  The Linkage, in CSV mode.

         Linkage::Init does the whole cyclic bring-up: it remaps the PDOs
         for this mode, walks the nodes through Pre-Op, configures the sync
         managers, confirms the setpoints are seeded and brings everything
         back to Operational.  Every step of it is in the log.
    -----------------------------------------------------------------------*/
   Amp *ampPtr[AXIS_COUNT];
   for( int i=0; i<AXIS_COUNT; i++ ) ampPtr[i] = &amp[i];

   err = link.Init( AXIS_COUNT, ampPtr, LINKAGE_CYCLIC_CSV );
   showerr( err, "initializing the linkage in CSV mode" );

   err = link.SetMoveLimits( MAX_VEL, MAX_ACC, MAX_DEC, MAX_JRK );
   showerr( err, "setting the move limits" );

   /*-----------------------------------------------------------------------
     4.  Enable.  In mode 9 the hold is a commanded ZERO VELOCITY, which is
         a genuine stop - the drive holds the axis against its velocity
         loop.
    -----------------------------------------------------------------------*/
   err = link.EnableAndHold();
   showerr( err, "enabling the linkage" );

   printf( "%s enabled, holding zero velocity.\n", AXES_TEXT );

   /*-----------------------------------------------------------------------
     5.  Back and forth until a key is pressed.  Same calls as mode 8 - the
         difference is on the wire.

         The key calls HaltMove from the key thread, wherever the stroke
         has got to.  HaltMove ramps both axes to zero together and KEEPS
         SENDING ZERO.  It does not stop streaming - stopping the stream
         here would leave the drives running at whatever velocity they last
         received.
    -----------------------------------------------------------------------*/
   haltLink = &link;
   StartKeyWatcher( HaltOnKey );

   printf( "\nMoving back and forth by %d.  Press any key to halt.\n\n",
           (int)MOVE_DISTANCE );

   Point<AXIS_COUNT> target;
   int  dir     = 1;
   long strokes = 0;

   for( ;; )
   {
      {
         std::lock_guard<std::mutex> lock( moveMtx );
         if( keyPressed ) break;

         for( int i=0; i<AXIS_COUNT; i++ )
            target[i] = dir * AxisSign( i ) * MOVE_DISTANCE;

         err = link.MoveToRel( target );
      }
      showerr( err, "starting a stroke" );

      /* One look at the velocity part way through the stroke.
         GetCsvActualVelAtomic reads the cyclic feedback PDO, so this costs
         nothing and takes no lock. */
      Thread::sleep( MID_STROKE_MS );

      int32 vel[AXIS_COUNT];
      for( int i=0; i<AXIS_COUNT; i++ )
         vel[i] = amp[i].GetCsvActualVelAtomic();

      err = link.WaitMoveDone( STROKE_TIMEOUT_MS );
      showerr( err, "waiting for a stroke" );

      const uint32 worstUs = AddStroke( link );

      if( keyPressed ) break;        /* this was the stroke the key halted */

      strokes++;
      printf( "  stroke %4ld done, velocity part way:", strokes );
      for( int i=0; i<AXIS_COUNT; i++ )
         printf( "   axis %d %8d", i + 1, (int)vel[i] );
      printf( "   worst period %u us\n", (unsigned)worstUs );

      dir = -dir;
      Thread::sleep( STROKE_DWELL_MS );
   }

   printf( "Halted, after %ld complete strokes, and still streaming zero.\n",
           strokes );

   for( int i=0; i<AXIS_COUNT; i++ )
      printf( "  axis %d actual velocity %d, feedback seen: %s\n", i + 1,
              (int)amp[i].GetCsvActualVelAtomic(),
              amp[i].CsvFeedbackSeen() ? "yes" : "NO - no frame received" );

   showRunTotals();


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

   printf( "\nDone.  The whole bring-up and run is in csv_cml.log.\n" );
   return 0;
}
