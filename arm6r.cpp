/** \file

CML example: six-axis arm from a URDF, pick-and-place, PVT or CSP

A six-axis articulated arm of the 5 kg-class cobot size, described by the
URDF file arm6r.urdf beside this one, in meters and radians, as URDF is.
The arm is imported from it with UrdfImportKin6R, which derives the DH
table and checks it against the file, and the keep-out model is built from
the same file's collision shapes with UrdfImportKeepOut.  The tool point is
the file's tool0 link, the flange.

It runs a pick-and-place: down to the pick, up, over the fixture to the
place with the corners rounded and the tool turning a quarter turn, down,
up, and back.  The four slots of a 2 x 2 place pattern are filled in turn,
so consecutive cycles are not the same moves.  Before the cycle it plans
one move straight into the fixture, to show the guard refusing it before
anything is sent.

Speeds are set for an unloaded arm on a bench.  Raise them only after
a PLAN_ONLY run passes with your own limits.

HOW THIS EXAMPLE RUNS
---------------------
With PLAN_ONLY commented out, it brings up the network and one drive axis
per joint, gives each axis the joint's own unit (radians for a revolute
joint, millimeters for a prismatic one) and the joint's own limits, and
enables the axes where they stand.

It then reads where the joints are, plans a jerk-limited joint move from
there to the start pose, and runs it.  After that it runs the cycle until
you press a key.  Every move is planned from the end of the one before it,
checked against the keep-out scene before it is sent, and sent only if it
clears.  The next move is planned while the current one runs, so the robot
does not pause between moves.

The key halts the robot.  The example says whether the halt came in the
middle of a move or between two, where the robot came to rest and how the
run went, then disables the axes.

Each drive's position must read zero where the model's joint is zero, in
the joint unit set by COUNTS_PER_UNIT: home the robot, or set the position
offsets with CME, before the first run.

PVT OR CSP: ONE SWITCH
----------------------
STREAM_CSP, in YOUR MACHINE below, picks how the moves reach the drives.

    defined       cyclic synchronous position (DS402 mode 8).  The linkage
                  resamples each move onto the network cycle and sends every
                  drive one position per cycle.
    commented     PVT.  The drives interpolate the points themselves and the
    out           linkage keeps their buffers filled.

The plans are the same in both modes.  A Cartesian move is planned with
CartPathPlanner::PlanTimedSCurve (PlanBlendedSCurve for the transfers,
whose corners are rounded and passed at speed) into a PvtConstAccelTrj, and a joint move with
PlanJointMove into a LinkSampledTrj.  Both are trajectories that
Linkage::SendTrajectory streams in either mode, so this file has one set of
plans rather than a second set built with PlanCspSCurve for CSP.  The switch
changes Linkage::Init and, in PVT, turns on SetPvtStartCheck: every plan
here starts from the commanded position, so a trajectory that does not is a
mistake to refuse.

A halt differs between the two:

    CSP   the linkage ramps every joint to rest at its JOINT_HALT_DEC, all
          arriving at the same instant.
    PVT   each drive stops its own axis at its JOINT_HALT_DEC (halt mode
          HALT_DECEL, set below), so the joints do not stop together.

In both the tool leaves the planned path while it stops.  Neither is an
emergency stop: keep the machine's own safety circuit in place.

KEEP-OUT
--------
BuildGuard, below, describes the cell: the arm, the table, the fixture and
the post, all from arm6r.urdf, and the robot's own mounting plate and base
column.  Every planned move is checked against it, including the motion
between the planned points, and a move that would come within KEEPOUT_MARGIN
of any of it is not sent.  Edit arm6r.urdf, and the zones in YOUR MACHINE,
to match your cell before running anything.  The check is a planning aid,
not a safety-rated component.

CHECK YOUR EDITS FIRST: PLAN_ONLY
---------------------------------
PLAN_ONLY, just below the transport switch, is defined as shipped.  The
program then opens no network and moves nothing.  It plans the whole
program from PLAN_ONLY_SEED, the move to the start pose and then
PLAN_ONLY_CYCLES cycles, and checks every plan against the keep-out scene.
It streams each one through the same objects the real run sends, in the
mode STREAM_CSP selects: in PVT the points are evaluated as the drive
interpolates them, and in CSP they are resampled and checked as the linkage
does every cycle.  It prints each joint's peak velocity, acceleration and
jerk against its limit, how far the streamed tool strays from the planned
path, any CSP setpoint the per-cycle check would hold, and PASS or FAIL,
and exits 0 on a pass.  Run it after every change to YOUR MACHINE or to
BuildGuard, in the mode you will run, and comment PLAN_ONLY out only when
it passes.

RobotPlanCheck (CML_RobotCell.h) does the whole check: it plans each move,
checks it against the keep-out scene, measures how close it comes to each
obstacle and streams it as the drives would receive it.  A program of your
own can run the same check on its moves before it sends them.

WHICH TRANSPORT
---------------
USE_CAN is defined just below.  LEAVE IT IN for a CANopen network; COMMENT
IT OUT for EtherCAT.

    CANopen    one node per joint.  A multi-axis CANopen drive presents
               each axis as its own node, so every joint uses Amp::Init
               with its own node ID.
    EtherCAT   one node per drive.  The joint on axis 1 of a drive uses
               Amp::Init; the joints on its other axes use InitSubAxis on
               that joint's Amp.

EMBEDDED LINUX / PREEMPT_RT
---------------------------
Everything a real-time deployment needs is in the SetUpRealTime() section
below, commented out with the reasoning.  Uncomment what applies to your
system.  Nothing in this example requires it, but a cyclic stream on a
loaded machine drops cycles without it, and the statistics at the end are
how you tell.

BUILD
-----
On Linux, from this folder:

    make arm6r

which builds the CML library first if it needs building.  Or by hand, from
the top of the CML tree:

    g++ -O2 -g -I inc -I inc/can -I inc/ecat \
        -o arm6r examples/arm6r/arm6r.cpp libMotionLib.a -lpthread -lrt -ldl

On Windows, see readme.txt in this folder.

Run it from a terminal, from wherever your CML log should land; it writes
arm6r_cml.log.
*/

/***************************************************************************
 *  TRANSPORT.  Leave this defined for CANopen; comment it out for EtherCAT.
 ***************************************************************************/
#define USE_CAN

/***************************************************************************
 *  PLAN ONLY.  Leave this defined to plan and check the whole program with
 *  no network and no drives; comment it out to run the robot.
 ***************************************************************************/
#define PLAN_ONLY

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <string>
#include <fstream>
#include <sstream>

#include "CML.h"
#include "CML_Kin6R.h"
#include "CML_UrdfImporter.h"
#include "CML_KeepOut.h"
#include "CML_RobotCell.h"

#if defined( USE_CAN )
#  include "can/can_copley.h"        /* or the header for your CAN card */
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

/* How the moves are streamed.  Leave this defined for cyclic synchronous
   position (mode 8); comment it out for PVT. */
#define STREAM_CSP

/* The robot's joint count and the coordinates in its Cartesian pose. */
#define JOINTS   6
#define POSE_DIM 6

#if defined( USE_CAN )

/* One CANopen node per joint, in joint order. */
static const int16 JOINT_NODE_ID[JOINTS] = { 1, 2, 3, 4, 5, 6 };

/* CAN port and bit rate.  The bit rate is in bits per second. */
static const char *CAN_PORT = "CAN0";
static const int32 CAN_BPS  = 1000000;

/* SYNC period, in MICROSECONDS.  In CSP this is the cyclic rate.  10000us
   = 100 Hz is a safe start for this many axes on 1 Mbit; come down from
   there once the bus load is known. */
static const uint32 SYNC_PERIOD_US = 10000;

#else

/* Each joint's drive and the axis it is on.  A drive is named by its
   position on the network: -1 is the first node, -2 the second, and so on.
   The joint on axis 1 of a drive must be listed before the joints on its
   other axes. */
static const int16 JOINT_DRIVE[JOINTS] = { -1, -1, -2, -2, -3, -3 };
static const int   JOINT_AXIS [JOINTS] = {  1,  2,  1,  2,  1,  2 };

/* The NIC.  On Windows this is the IP the UDP transport binds to; on Linux
   it is the interface name. */
#  if defined( WIN32 )
static const char *ECAT_WIN_IP    = "192.168.0.33";
#  else
static const char *ECAT_INTERFACE = "eth0";
#  endif

/* Cycle period in MILLISECONDS.  5 ms holds on a desktop Windows timer;
   a real-time system can go much lower. */
static const int ECAT_CYCLE_MS = 5;

#endif

/* Encoder counts per joint unit.  This makes every axis's user unit the
   joint's kinematic unit: radians for a revolute joint, millimeters for a
   prismatic one.  For a revolute joint it is counts per motor turn times
   the gear ratio, divided by 2*pi. */
static const double COUNTS_PER_UNIT[JOINTS] = { 2.107e6, 2.107e6, 2.107e6, 2.107e6, 2.107e6, 2.107e6 };

/* Each joint's travel, in its own unit.  The planner refuses any plan that
   needs a joint outside it. */
static const double JOINT_MIN[JOINTS] = { -2*M_PI, -M_PI, -M_PI, -2*M_PI, -2*M_PI, -2*M_PI };
static const double JOINT_MAX[JOINTS] = {  2*M_PI,  0.0,   M_PI,  2*M_PI,  2*M_PI,  2*M_PI };

/* Each joint's velocity, acceleration and jerk limit, per second, and the
   deceleration a halt uses.  Every plan stays inside the first three, and
   the linkage checks every setpoint against the velocity limit. */
static const double JOINT_VEL     [JOINTS] = {  1.5,  1.5,  1.5,  2.0,  2.0,  2.0 };
static const double JOINT_ACC     [JOINTS] = {  4.0,  4.0,  4.0,  8.0,  8.0,  8.0 };
static const double JOINT_JERK    [JOINTS] = { 40.0, 40.0, 40.0, 80.0, 80.0, 80.0 };
static const double JOINT_HALT_DEC[JOINTS] = {  6.0,  6.0,  6.0, 12.0, 12.0, 12.0 };

/* The tool's speed and acceleration along a Cartesian move, meters, and
   how fast it may turn, radians, per second.  The joint limits above slow
   a move further wherever they bind. */
static const double CART_VEL     = 0.4;         /* m/s      */
static const double CART_ACC     = 1.5;         /* m/s^2    */
static const double CART_ROT_VEL = 1.5;         /* rad/s    */
static const double CART_ROT_ACC = 6.0;         /* rad/s^2  */

/* The spacing of the planned points, in milliseconds: the PVT point time,
   and the spacing the CSP stream resamples from. */
static const int PVT_POINT_MS = 4;

/* The robot description: the URDF file, read from the current folder or
   else from the program's, and the link that is the tool point.  Its
   position limits are replaced by JOINT_MIN and JOINT_MAX above.
   READY_JOINTS is a pose clear of the cell: it picks the arm's solution
   branch, and the guard is set up there. */
static const char  *URDF_FILE = "arm6r.urdf";
static const char  *URDF_TIP  = "tool0";
static const double READY_JOINTS[JOINTS] = { 0.0, -1.9, 1.9, -1.57, -1.57, 0.0 };

/* The robot's mounting plate, its half-width and thickness, and its base
   column, meters.  The column is a capsule on the base axis, so it reaches
   COLUMN_RADIUS above COLUMN_TOP: these hold the base housing and leave
   the upper arm room to pass above it. */
static const double MOUNT_HALF      = 0.40;
static const double MOUNT_THICKNESS = 0.02;
static const double COLUMN_RADIUS   = 0.070;
static const double COLUMN_TOP      = 0.030;

/* How close two parts of the arm may come to each other, meters. */
static const double SELF_MARGIN = 0.005;

/* The cell, meters in the robot's base frame, with the tool pointing down.
   PICK_XY is the pick and PLACE_XY the center of a 2 x 2 pattern of place
   slots SLOT_PITCH apart, either side of the fixture.  The flange is at
   WORK_Z at a pick or a place, APPROACH_Z above one, and TRANSFER_Z over
   the fixture, where the corners are rounded to TRANSFER_BLEND.  The tool
   turns PLACE_YAW about the vertical between the pick and the place.
   INTO_FIXTURE is a point inside the fixture, the end of the move the guard
   must refuse. */
static const double PICK_XY[2]      = { 0.45, -0.30 };
static const double PLACE_XY[2]     = { 0.45,  0.30 };
static const double SLOT_PITCH      = 0.08;
static const double WORK_Z          = 0.10;
static const double APPROACH_Z      = 0.25;
static const double TRANSFER_Z      = 0.35;
static const double TRANSFER_BLEND  = 0.08;
static const double PLACE_YAW       = M_PI/2;
static const double INTO_FIXTURE[3] = { 0.475, 0.0, 0.10 };

/* How close any part of the robot may come to anything in BuildGuard's
   scene, in meters. */
static const double KEEPOUT_MARGIN = 0.005;

/* PLAN_ONLY: where the plan starts, in joint units, and how many cycles
   it plans after the move to the start pose. */
static const double PLAN_ONLY_SEED[JOINTS] = { -0.9, -1.6, 1.5, -1.4, -1.57, 0.3 };
static const int    PLAN_ONLY_CYCLES = 4;


/***************************************************************************
 *  Moves
 ***************************************************************************/

typedef Point<JOINTS>   Joints;
typedef Point<POSE_DIM> Pose;

/* How every move is planned: the spacing of its points, and the tool's
   speed and acceleration along a Cartesian move.  The joint limits are the
   arm's own, set in SetUpArm. */
static const RobotMoveLimits MOVE_LIMITS =
   { PVT_POINT_MS / 1000.0, CART_VEL, CART_ACC, CART_ROT_VEL, CART_ROT_ACC };

static void ToJoints( const double q[], Joints &p )
{
   p.setDim( JOINTS );
   for( int i=0; i<JOINTS; i++ ) p[i] = q[i];
}

/* What AddRobotZones added, in the robot's length unit times 'scale' mm. */
static void ShowZones( const KeepOutRobotZones &z, bool selfCheck, double scale )
{
   const char *lead = "  zones :";

   if( z.mount )
   {
      printf( "%s a mount %s z = %.1f mm, %.0f mm square\n", lead,
              ( z.mountSide > 0 ) ? "below" : "above", z.mountZ * scale,
              2 * z.mountHalfSize * scale );
      lead = "         ";
   }
   if( z.column )
   {
      printf( "%s a column of radius %.1f mm from z = %.1f to %.1f mm\n", lead,
              z.columnRadius * scale, z.columnBottom * scale, z.columnTop * scale );
      lead = "         ";
   }
   printf( "%s self-collision checking %s\n\n", lead,
           selfCheck ? "ON" : "off: the helper does not enable it for this robot" );
}

/* argv[0], for finding files that ship beside the program. */
static const char *programPath = "";

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


/***************************************************************************
 *  The arm, the cell and the cycle, all from the URDF
 ***************************************************************************/
static Kin6R        arm;
static KeepOutGuard guard;

/* The arm is in meters and radians, as its URDF is; the reports print mm. */
static const double MM_PER_UNIT = 1000.0;

static const char *JOINT_NAME[JOINTS]   = { "J1", "J2", "J3", "J4", "J5", "J6" };
static const char *JOINT_UNIT[JOINTS]   = { "rad", "rad", "rad", "rad", "rad", "rad" };
static const char *POSE_NAME[POSE_DIM]  = { "x", "y", "z", "yaw", "pitch", "roll" };
static const bool  POSE_ANGLE[POSE_DIM] = { false, false, false, true, true, true };

/* The robot zones' ids; the URDF's obstacles take their shapes' indices in
   the file, which are smaller. */
enum { OBST_MOUNT = 100, OBST_COLUMN = 101 };

/* The URDF, read once.  Both imports and the names in the reports come
   from it. */
static std::string urdfXml;
static UrdfModel   urdfModel;
static char        partName[JOINTS + 2][2 * URDF_MAX_NAME];

static const char *UrdfStatName( UrdfStat s )
{
   switch( s )
   {
      case URDF_OK:                return "OK";
      case URDF_XML_ERROR:         return "not readable XML";
      case URDF_NO_ROBOT:          return "no robot in it";
      case URDF_TOO_MANY:          return "more joints or shapes than the importer holds";
      case URDF_CHAIN_NOT_FOUND:   return "URDF_TIP is not reachable from the root";
      case URDF_UNSUPPORTED_JOINT: return "a joint in the chain that a six-axis arm cannot have";
      case URDF_WRONG_DOF:         return "the chain does not have six moving joints";
      case URDF_DH_FAILED:         return "the joint axes admit no DH table";
      case URDF_VERIFY_FAILED:     return "the derived DH table does not reproduce the file";
      case URDF_BAD_NAME:          return "a name too long, or used twice";
      case URDF_BAD_VALUE:         return "a value that is not a usable number";
      case URDF_UNCOVERED:         return "a link with no collision shape keep-out can model";
   }
   return "unknown";
}

/* Read URDF_FILE from the current folder, or else from the program's. */
static bool ReadUrdf( void )
{
   std::string path = URDF_FILE;
   std::ifstream f( path.c_str(), std::ios::binary );

   if( !f )
   {
      const std::string prog = programPath;
      const size_t slash = prog.find_last_of( "/\\" );
      if( slash != std::string::npos )
      {
         path = prog.substr( 0, slash + 1 ) + URDF_FILE;
         f.open( path.c_str(), std::ios::binary );
      }
   }
   if( !f )
   {
      printf( "ERROR: cannot open %s, here or beside the program.\n", URDF_FILE );
      return false;
   }

   std::stringstream text;
   text << f.rdbuf();
   urdfXml = text.str();
   printf( "Robot description %s:\n", path.c_str() );
   return true;
}

/* The arm, imported from the URDF.  The importer derives a DH table from
   the file's joints and refuses the import unless that table reproduces
   the file's own forward kinematics. */
static bool SetUpGeometry( void )
{
   if( !ReadUrdf() ) return false;

   UrdfImportReport rep;
   memset( &rep, 0, sizeof( rep ) );

   const UrdfStat st = UrdfImportKin6R( urdfXml.c_str(), URDF_TIP, arm, &rep );
   if( st != URDF_OK )
   {
      printf( "ERROR: the URDF did not import as a six-axis arm: %s.\n", UrdfStatName( st ) );
      return false;
   }

   printf( "  arm   : %d moving joints in a chain of %d; the DH table matches the file to %.1e\n",
           rep.movableJoints, rep.chainLength, rep.verifyError );
   printf( "          joint limits in the file: %s (YOUR MACHINE's apply)\n",
           rep.limitsApplied ? "yes" : "none" );

   /* The names the reports use: each body part is labeled with the chain
      link it rides on, 0 being the base; the tool point is the last. */
   if( UrdfParse( urdfXml.c_str(), urdfModel ) != URDF_OK ) return false;

   int order[URDF_MAX_JOINTS], n = 0;
   if( UrdfBuildChain( urdfModel, URDF_TIP, order, n ) != URDF_OK ) return false;

   int label = 0;
   snprintf( partName[0], sizeof( partName[0] ), "%s", urdfModel.joint[ order[0] ].parent );
   for( int k=0; k<n; k++ )
   {
      const UrdfJoint &jt = urdfModel.joint[ order[k] ];
      if( jt.type != URDF_JOINT_FIXED )
      {
         if( ++label > JOINTS ) return false;
         snprintf( partName[label], sizeof( partName[label] ), "%s", jt.child );
      }
      else
      {
         const size_t used = strlen( partName[label] );
         snprintf( partName[label] + used, sizeof( partName[label] ) - used, " + %s", jt.child );
      }
   }
   snprintf( partName[JOINTS + 1], sizeof( partName[JOINTS + 1] ), "tool point" );
   return true;
}

static const char *BodyPartName( int part )
{
   return ( ( part >= 0 ) && ( part <= JOINTS + 1 ) ) ? partName[part] : "arm";
}

static const char *ObstacleName( int id )
{
   if( id == OBST_MOUNT )  return "mount";
   if( id == OBST_COLUMN ) return "column";
   if( ( id >= 0 ) && ( id < urdfModel.ngeom ) ) return urdfModel.geom[id].link;
   return "obstacle";
}

/***************************************************************************
 *  KEEP-OUT
 *
 *  Built from the same URDF as the arm.  UrdfImportKeepOut turns every
 *  collision shape on a moving link into a capsule carried with that link,
 *  and every shape on a link that does not move - the base, and the table,
 *  fixture and post the file fixes to it - into an obstacle, each exempting
 *  the parts that sit in it by design.  A mesh cannot be turned into a
 *  capsule, so the report says how many were skipped: a URDF of meshes alone
 *  imports with no volumes at all.
 *
 *  AddRobotZones then adds the robot's own mounting plate and base column,
 *  sized below, and turns on self-collision checking where the body model
 *  supports it, which is what the URDF's shapes give it.
 *
 *  Sweeping is on: between two planned points the guard subdivides until
 *  no part of the arm moves more than SWEEP_STEP, so a link cannot pass
 *  through the fixture between two points.
 ***************************************************************************/
static const double SWEEP_STEP = 0.005;    /* m */

static bool BuildGuard( void )
{
   Joints ref;
   ToJoints( READY_JOINTS, ref );

   guard.SetMargin( KEEPOUT_MARGIN );
   if( !guard.SetSelfMargin( SELF_MARGIN ) || !guard.SetMaxSweepStep( SWEEP_STEP ) )
      return false;

   UrdfImportReport rep;
   memset( &rep, 0, sizeof( rep ) );

   const UrdfStat st = UrdfImportKeepOut( urdfXml.c_str(), URDF_TIP, arm, ref, guard, &rep );
   if( st != URDF_OK )
   {
      printf( "ERROR: the URDF's collision shapes did not import: %s.\n", UrdfStatName( st ) );
      return false;
   }

   printf( "  guard : %d shapes on %d moving links, %d cell obstacles\n",
           rep.attachments, rep.linksWithGeom, rep.sceneObstacles );
   printf( "          skipped: %d mesh(es), %d unknown shape(s), %d behind joints off the chain\n",
           rep.meshSkipped, rep.unknownSkipped, rep.branchSkipped );

   KeepOutRobotZones z;
   if( guard.DefaultRobotZones( arm, ref, z ) != KEEPOUT_OK ) return false;

   z.mount          = true;
   z.mountHalfSize  = MOUNT_HALF;
   z.mountThickness = MOUNT_THICKNESS;
   z.column         = true;
   z.columnRadius   = COLUMN_RADIUS;
   z.columnBottom   = 0;
   z.columnTop      = COLUMN_TOP;

   if( guard.AddRobotZones( arm, ref, z, OBST_MOUNT ) != KEEPOUT_OK )
   {
      printf( "ERROR: the mount and column do not fit the arm at READY_JOINTS.\n" );
      return false;
   }

   ShowZones( z, guard.SelfCheck(), MM_PER_UNIT );
   return true;
}

/* The tool pointing straight down, turned by 'yaw' about the vertical. */
static void ToolDown( double x, double y, double z, double yaw, double p[] )
{
   p[0] = x;   p[1] = y;   p[2] = z;
   p[3] = yaw; p[4] = 0;   p[5] = M_PI;
}

static void CycleStartPose( double p[] )
{
   ToolDown( PICK_XY[0], PICK_XY[1], APPROACH_Z, 0, p );
}

/* The move the guard must refuse: from the start pose straight down into
   the fixture.  It is planned and checked, and never sent. */
static bool RefusalSpec( const double from[], RobotMoveSpec &m )
{
   (void)from;

   double a[POSE_DIM], b[POSE_DIM];
   CycleStartPose( a );
   ToolDown( INTO_FIXTURE[0], INTO_FIXTURE[1], INTO_FIXTURE[2], 0, b );
   m.Line( "into the fixture", a, b );
   return true;
}

/* The cycle: down to the pick, up, over the fixture to a place slot, down,
   up, and back over the fixture.  The four place slots are filled in turn. */
static const int CYCLE_MOVES = 6;

static void CycleMove( long k, const double from[], RobotMoveSpec &m )
{
   (void)from;

   const int    slot = (int)( ( k / CYCLE_MOVES ) % 4 );
   const double sx = PLACE_XY[0] + ( ( slot & 1 ) ? 0.5 : -0.5 ) * SLOT_PITCH;
   const double sy = PLACE_XY[1] + ( ( slot & 2 ) ? 0.5 : -0.5 ) * SLOT_PITCH;
   const double px = PICK_XY[0], py = PICK_XY[1];

   double a[POSE_DIM], b[POSE_DIM], c[POSE_DIM], d[POSE_DIM];

   switch( (int)( k % CYCLE_MOVES ) )
   {
      case 0:
         ToolDown( px, py, APPROACH_Z, 0, a );
         ToolDown( px, py, WORK_Z, 0, b );
         m.Line( "down to the pick", a, b );
         break;

      case 1:
         ToolDown( px, py, WORK_Z, 0, a );
         ToolDown( px, py, APPROACH_Z, 0, b );
         m.Line( "up from the pick", a, b );
         break;

      case 2:
         /* Up, across and down in one move, the corners rounded, the tool
            turning to PLACE_YAW on the way. */
         m.Path( "over to the place", TRANSFER_BLEND );
         ToolDown( px, py, APPROACH_Z, 0, a );
         ToolDown( px, py, TRANSFER_Z, 0, b );
         ToolDown( sx, sy, TRANSFER_Z, PLACE_YAW, c );
         ToolDown( sx, sy, APPROACH_Z, PLACE_YAW, d );
         m.AddWay( a ); m.AddWay( b ); m.AddWay( c ); m.AddWay( d );
         break;

      case 3:
         ToolDown( sx, sy, APPROACH_Z, PLACE_YAW, a );
         ToolDown( sx, sy, WORK_Z, PLACE_YAW, b );
         m.Line( "down to the place", a, b );
         break;

      case 4:
         ToolDown( sx, sy, WORK_Z, PLACE_YAW, a );
         ToolDown( sx, sy, APPROACH_Z, PLACE_YAW, b );
         m.Line( "up from the place", a, b );
         break;

      default:
         m.Path( "back to the pick", TRANSFER_BLEND );
         ToolDown( sx, sy, APPROACH_Z, PLACE_YAW, a );
         ToolDown( sx, sy, TRANSFER_Z, PLACE_YAW, b );
         ToolDown( px, py, TRANSFER_Z, 0, c );
         ToolDown( px, py, APPROACH_Z, 0, d );
         m.AddWay( a ); m.AddWay( b ); m.AddWay( c ); m.AddWay( d );
         break;
   }
}

/* The program: a joint move to the cycle's first pose, solved on the
   branch READY_JOINTS is on, the move the guard must refuse, then the
   cycle.  The plan-only check and the run both take their moves from it. */
static const RobotProgram PROGRAM =
   { "to the start pose", CycleStartPose, READY_JOINTS, RefusalSpec, CycleMove, CYCLE_MOVES };

/***************************************************************************
 *  Planning
 ***************************************************************************/

static bool SetUpArm( void )
{
   if( !SetUpGeometry() ) return false;

   Joints lo, hi, v, a, j;
   ToJoints( JOINT_MIN, lo );
   ToJoints( JOINT_MAX, hi );
   ToJoints( JOINT_VEL, v );
   ToJoints( JOINT_ACC, a );
   ToJoints( JOINT_JERK, j );

   arm.SetJointLimits( lo, hi );
   arm.SetJointVelLimits( v );
   arm.SetJointAccLimits( a );
   arm.SetJointJerkLimits( j );
   return true;
}

/* The arm in its cell: every move is planned with MOVE_LIMITS and checked
   against the guard, and the reports name things as below. */
static const RobotNames NAMES =
   { JOINT_NAME, JOINT_UNIT, POSE_NAME, POSE_ANGLE, BodyPartName, ObstacleName, MM_PER_UNIT };

static RobotCell cell;

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

/* The halt the key triggers.  The main loop sends every move holding
   moveMtx and marks it running there, and this takes moveMtx too, so the
   key either finds a move running and halts it, or finds none and the loop
   sends no other. */
static std::mutex   moveMtx;
static Linkage     *haltLink   = 0;
static const char  *runningMove = 0;     /* the move streaming now, or NULL */
static const char  *haltedMove  = 0;     /* the move the key halted, or NULL */

static void HaltOnKey( void )
{
   std::lock_guard<std::mutex> lock( moveMtx );

   if( !runningMove )
   {
      printf( "\nKey pressed between moves - no move is sent after this.\n" );
      return;
   }

   haltedMove = runningMove;
   printf( "\nKey pressed during '%s' - halting mid-move.\n", runningMove );

   const Error *err = haltLink->HaltMove();
   if( err ) printf( "WARNING: HaltMove failed: %s\n", err->toString() );
}

/***************************************************************************
 *  REAL-TIME SETUP FOR EMBEDDED LINUX
 *
 *  Every line in this function is commented out.  None of it is needed to
 *  make the example run; all of it matters on a real machine at a real
 *  cycle rate.  Uncomment what applies and leave the rest.
 *
 *  The numbers that tell you whether it worked are the ones the run
 *  statistics print.
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
         callback processing away.
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
     3.  AFFINITY.  Pin the cyclic work to the isolated core.  -1, the
         default, leaves placement to the scheduler, which will migrate the
         thread off a warm cache at the worst possible moment.
    =======================================================================*/
#if !defined( USE_CAN )
   /* settings.cycleThreadAffinity = 4; */      /* an isolated core */
   /* settings.readThreadAffinity  = -1; */     /* fine on the general pool */
#endif

   /*  CSP only: the setpoint stream's own thread, which is a Linkage thread
       rather than a network thread.  Takes effect at the start of the next
       move, because that is when the thread is created.  Pin it to a
       DIFFERENT core from the cycle thread unless that core is genuinely
       idle: the two run at the same rate and will fight. */
   /* link.SetCspThreadAffinity( 5 ); */

   /*=======================================================================
     4.  BUSY-WAIT THE LAST FEW MICROSECONDS.  EtherCAT only.

         Waking deliberately early and spinning the remainder takes the
         scheduler's wake-up latency out of the timing path.  It burns the
         core for the whole spin: pin the cycle thread first (step 3).

             > 0   sleep until this many us before the deadline, then spin.
             < 0   spin the ENTIRE interval; only on an isolated core.
    =======================================================================*/
#if !defined( USE_CAN )
   /* settings.cycleThreadSpinUs = 50; */
#endif

   /*=======================================================================
     5.  LOCK MEMORY.  A page fault in the cyclic path costs milliseconds.

             #include <sys/mman.h>
             mlockall( MCL_CURRENT | MCL_FUTURE );

         Do this once, early, before the network is opened.
    =======================================================================*/

   /*=======================================================================
     6.  WHAT NOT TO DO IN THE CYCLIC PATH

         No file I/O, no logging, no allocation, no mutex shared with a
         non-real-time thread.  CML's own cyclic path takes none of those,
         and this example plans on the main thread only.
    =======================================================================*/
}

static int PlanOnly( void );

#if defined( PLAN_ONLY )
static const bool planOnly = true;
#else
static const bool planOnly = false;
#endif

/***************************************************************************
 *  main
 ***************************************************************************/
int main( int argc, char **argv )
{
   /* PLAN_ONLY alone chooses the run.  Any argument is refused, so none can
      start the robot by accident. */
   if( argc > 1 )
   {
      printf( "usage: %s\n  This example takes no arguments.  PLAN_ONLY, at the top of\n"
              "  arm6r.cpp, chooses a plan-only run.\n", argv[0] );
      return 2;
   }

   programPath = argv[0];
   if( !SetUpArm() || !BuildGuard() ) return 1;
   showerr( cell.Setup( arm, &guard, MOVE_LIMITS, NAMES ), "setting up the cell" );

   /* PLAN_ONLY: plan and check the program, and stop.  The run below is
      compiled either way, so commenting PLAN_ONLY out changes nothing else. */
   if( planOnly ) return PlanOnly();

   /* LOG_EVERYTHING while you are bringing a machine up.  It is the single
      most useful thing to send with a support request, and it is what the
      bring-up sequence writes its findings to. */
   cml.SetLogFile( "arm6r_cml.log" );
   cml.SetDebugLevel( LOG_EVERYTHING );

   printf( "CML example: six-axis arm from a URDF, pick-and-place, PVT or CSP\n" );
#if defined( STREAM_CSP )
   printf( "CSP (mode 8), " );
#else
   printf( "PVT, " );
#endif
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
     2.  One axis per joint, each in the joint's own unit.
    -----------------------------------------------------------------------*/
   Amp  amp[JOINTS];
   char what[80];

#if defined( USE_CAN )

   /* On CANopen the SYNC period is an AMP setting rather than a network
      one.  Every axis must agree. */
   AmpSettings ampSettings;
   ampSettings.synchPeriod = SYNC_PERIOD_US;

   for( int i=0; i<JOINTS; i++ )
   {
      snprintf( what, sizeof( what ), "initializing joint %s", JOINT_NAME[i] );
      err = amp[i].Init( net, JOINT_NODE_ID[i], ampSettings );
      showerr( err, what );
   }

#else

   for( int i=0; i<JOINTS; i++ )
   {
      snprintf( what, sizeof( what ), "initializing joint %s", JOINT_NAME[i] );

      if( JOINT_AXIS[i] == 1 )
         err = amp[i].Init( net, JOINT_DRIVE[i] );
      else
      {
         /* Another axis of a drive already initialized for an earlier
            joint: a sub-axis of that joint's Amp, not a second Init. */
         int first = -1;
         for( int k=0; k<i; k++ )
            if( ( JOINT_DRIVE[k] == JOINT_DRIVE[i] ) && ( JOINT_AXIS[k] == 1 ) ) first = k;

         if( first < 0 )
         {
            printf( "ERROR: joint %s is on axis %d of a drive whose axis 1 is "
                    "not listed before it in JOINT_DRIVE.\n", JOINT_NAME[i], JOINT_AXIS[i] );
            return 1;
         }
         err = amp[i].InitSubAxis( amp[first], JOINT_AXIS[i] );
      }
      showerr( err, what );
   }

#endif

   for( int i=0; i<JOINTS; i++ )
   {
      snprintf( what, sizeof( what ), "setting the units of joint %s", JOINT_NAME[i] );
      err = amp[i].SetCountsPerUnit( COUNTS_PER_UNIT[i] );
      showerr( err, what );

#if !defined( STREAM_CSP )
      /* In PVT the drive halts its own axis.  Make that a deceleration at
         the joint's own rate rather than whatever the drive was left set
         to. */
      snprintf( what, sizeof( what ), "setting the halt of joint %s", JOINT_NAME[i] );
      err = amp[i].SetHaltMode( HALT_DECEL );
      if( !err ) err = amp[i].SetProfileDec( JOINT_HALT_DEC[i] );
      showerr( err, what );
#endif
   }

   /*-----------------------------------------------------------------------
     3.  The Linkage.

         Every axis gets its own limits: the joints differ in speed and in
         unit, and one scalar limit cannot describe them.  The velocity
         limit bounds each setpoint step in CSP and how far a trajectory
         may start from the commanded position in both modes.
    -----------------------------------------------------------------------*/
   Amp *ampPtr[JOINTS];
   for( int i=0; i<JOINTS; i++ ) ampPtr[i] = &amp[i];

#if defined( STREAM_CSP )
   err = link.Init( JOINTS, ampPtr, true );
   showerr( err, "initializing the linkage in CSP mode" );
#else
   err = link.Init( JOINTS, ampPtr, false );
   showerr( err, "initializing the linkage in PVT mode" );
   link.SetPvtStartCheck( true );
#endif

   for( int i=0; i<JOINTS; i++ )
   {
      snprintf( what, sizeof( what ), "setting the limits of joint %s", JOINT_NAME[i] );
      err = link.SetAxisLimits( (uint16)i, JOINT_VEL[i], JOINT_ACC[i], JOINT_HALT_DEC[i] );
      showerr( err, what );
   }

   /*-----------------------------------------------------------------------
     4.  Enable, and find out where the joints are.  The commanded
         position is where every plan starts, so it is the seed.
    -----------------------------------------------------------------------*/
   err = link.EnableAndHold();
   showerr( err, "enabling the linkage" );

   Joints cmd;
   cmd.setDim( JOINTS );
   err = link.GetPositionCommand( cmd );
   showerr( err, "reading the joint positions" );

   double seed[JOINTS];
   for( int i=0; i<JOINTS; i++ ) seed[i] = cmd[i];

   printf( "Enabled.  The joints are at\n  " );
   cell.PrintJoints( seed );
   printf( "\n" );

   for( int i=0; i<JOINTS; i++ )
   {
      if( ( seed[i] < JOINT_MIN[i] ) || ( seed[i] > JOINT_MAX[i] ) )
      {
         printf( "ERROR: joint %s is at %.4f %s, outside %.4f to %.4f.  Move it inside\n"
                 "its travel, or check its home and COUNTS_PER_UNIT.\n", JOINT_NAME[i],
                 seed[i], JOINT_UNIT[i], JOINT_MIN[i], JOINT_MAX[i] );
         for( int k=0; k<JOINTS; k++ ) amp[k].Disable();
         return 1;
      }
   }

   /*-----------------------------------------------------------------------
     5.  The cycle, after a joint move to the start pose.

         Two Move slots take turns: one is streaming while the next move is
         planned into the other, from where the streaming one ends.  A move
         goes out only if it planned and cleared the keep-out check.
    -----------------------------------------------------------------------*/
   haltLink = &link;
   StartKeyWatcher( HaltOnKey );

   static RobotMove slot[2];
   int  cur = 0;
   KeepOutCollisionInfo info;
   RobotMoveSpec spec( arm );
   RobotRunStats stats( CYCLE_MOVES );

   err = PROGRAM.Start( spec );
   if( err ) printf( "The start pose cannot be solved: %s\n", err->toString() );
   else
   {
      err = cell.Plan( spec, seed, slot[cur], &info );
      cell.Explain( spec, slot[cur], err, info );
   }
   if( err )
   {
      printf( "Nothing was sent.  Move the robot nearer the start pose and run again.\n" );
      for( int i=0; i<JOINTS; i++ ) amp[i].Disable();
      return 1;
   }

   printf( "\nMoving to the start pose, then running the cycle (%d moves).\n"
           "Press any key to halt.\n\n", CYCLE_MOVES );

   long idx = -1;                 /* slot[cur]'s place in the program; -1
                                     is the move to the start pose       */
   const RobotMove *last = 0;     /* the last move that completed          */
   const RobotMove *halted = 0;   /* the move the key halted               */
   double haltedAt = 0;           /* from its start to rest, in seconds    */
   bool   stopped = false;        /* the loop ended for another reason     */

   for( ;; )
   {
      RobotMove &m = slot[cur];

      /* Send it, unless the key came first. */
      {
         std::lock_guard<std::mutex> lock( moveMtx );
         if( keyPressed ) break;

         err = link.SendTrajectory( m.Trj() );
         if( !err ) runningMove = m.Name();
      }
      if( err )
      {
         printf( "ERROR sending '%s': %s\n", m.Name(), err->toString() );
         stopped = true;
         break;
      }
      stats.Sent( idx );

      /* Plan the next move while this one runs. */
      RobotMove &n = slot[1 - cur];
      RobotMoveSpec nextSpec( arm );
      PROGRAM.Move( idx + 1, m.End(), nextSpec );
      KeepOutCollisionInfo nextInfo;
      const Error *nextErr = cell.Plan( nextSpec, m.End(), n, &nextInfo );
      stats.Planned( n, m );

      /* Wait for this one. */
      err = link.WaitMoveDone( (Timeout)( m.Seconds() * 1000.0 ) + 10000 );
      {
         std::lock_guard<std::mutex> lock( moveMtx );
         runningMove = 0;
      }
      stats.Done( link, idx, !err && !haltedMove );

      if( haltedMove )
      {
         halted   = &m;
         haltedAt = stats.LastMoveSeconds();
         if( err ) printf( "  (the halted move ended with: %s)\n", err->toString() );
         break;
      }
      if( err )
      {
         printf( "ERROR waiting for '%s': %s\n", m.Name(), err->toString() );
         stopped = true;
         break;
      }
      last = &m;

      /* At the start pose, before the cycle: the program's move into the
         cell, planned and refused.  It is never sent. */
      if( idx < 0 )
      {
         RobotMoveSpec refusal( arm );
         if( PROGRAM.Refusal( m.End(), refusal ) && !cell.ShowRefusal( refusal, m.End() ) )
         {
            stopped = true;
            break;
         }
      }

      if( keyPressed ) break;          /* came between moves */

      if( nextErr )
      {
         printf( "Stopping: the next move was not sent.\n" );
         cell.Explain( nextSpec, n, nextErr, nextInfo );
         stopped = true;
         break;
      }

      idx++;
      cur = 1 - cur;
   }

   /*-----------------------------------------------------------------------
     6.  Where it stopped, how the run went, and disable.
    -----------------------------------------------------------------------*/
   if( halted )
      printf( "\nHalted MID-MOVE in '%s': at rest %.2f s after it started, in a move\n"
              "planned to take %.2f s.\n", halted->Name(), haltedAt, halted->Seconds() );
   else if( !stopped )
   {
      if( last ) printf( "\nStopped BETWEEN MOVES, at rest at the end of '%s'.\n", last->Name() );
      else       printf( "\nStopped before the first move.\n" );
   }

   err = link.GetPositionCommand( cmd );
   if( !err )
   {
      double at[JOINTS];
      for( int i=0; i<JOINTS; i++ ) at[i] = cmd[i];
      printf( "The robot is at\n  joints" );
      cell.PrintJoints( at );
      printf( "\n  tool  " );
      cell.PrintPose( at );
      printf( "\n" );
   }

   stats.ShowTotals();
   if( halted || last )
      RobotRunStats::ShowLastMove( link, halted ? "the move the key halted" : "the last move" );

   for( int i=0; i<JOINTS; i++ )
   {
      const Error *dErr = amp[i].Disable();

      if( dErr ) printf( "WARNING: joint %s would not disable: %s\n",
                         JOINT_NAME[i], dErr->toString() );
   }

   printf( "\nDone.  The whole bring-up and run is in arm6r_cml.log.\n" );
   return stopped ? 1 : 0;
}

/***************************************************************************
 *  PLAN_ONLY
 *
 *  The whole program planned from PLAN_ONLY_SEED and checked, without a
 *  network.  RobotPlanCheck::Run plans every move and checks it against the
 *  keep-out scene as the real run does, measures how close it comes to each
 *  obstacle and to the robot itself, and streams it as the drives would
 *  receive it, in the mode STREAM_CSP selects.  It reports each joint's
 *  peak velocity, acceleration and jerk against the arm's joint limits, how
 *  far the tool strays from the planned path, and the start check.
 ***************************************************************************/

/* How far past a limit a peak may go and pass.  The drive's cubic between
   two points can pass a jerk-limited profile's peak by a little. */
static const double CHECK_TOL = 0.02;

/* How far the streamed tool may stray from the planned path and pass: mm,
   and degrees of tool rotation. */
static const double PATH_TOL_MM  = 0.05;
static const double PATH_TOL_DEG = 0.05;

#if defined( STREAM_CSP )
static double CycleMs( void )
{
#  if defined( USE_CAN )
   return SYNC_PERIOD_US / 1000.0;
#  else
   return ECAT_CYCLE_MS;
#  endif
}
#endif

static int PlanOnly( void )
{
   static RobotPlanCheck check;

#if defined( STREAM_CSP )
   const Error *err = check.Setup( cell, TRJCHECK_CSP, CycleMs() );
#else
   const Error *err = check.Setup( cell, TRJCHECK_PVT );
#endif
   if( err )
   {
      printf( "ERROR: the trajectory check cannot be set up: %s\n", err->toString() );
      return 1;
   }
   check.SetTolerances( CHECK_TOL, PATH_TOL_MM / MM_PER_UNIT, PATH_TOL_DEG * M_PI / 180.0 );

   const bool pass = check.Run( "CML example: six-axis arm from a URDF, pick-and-place, PVT or CSP - plan only",
                                PROGRAM, PLAN_ONLY_SEED, PLAN_ONLY_CYCLES );
   return pass ? 0 : 1;
}
