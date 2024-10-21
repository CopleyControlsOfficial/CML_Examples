/*

StartPvtUsingDigitalInput.cpp

The following is an example of how to perform a PVT stream using CML when 
the drive is configured to initiate a PVT move using a digital input. In this
example, the drive is configured to begin the PVT stream when IN1 is pulled
high. 

NOTE: This example assumes a stepper motor is being driven. If you are not 
commanding a stepper motor, please change 

This example will configure IN1 to update the trajectory (start the move) when 
IN1 undergoes a low-high transition. It will also configure OUT1 to be Active ON 
when the trajectory generator is running (move in progress). 

CML will load the initial 32 PVT points into the drive's internal memory. Then it 
will wait for the state of IN1 to be high, meaning the move is in progress. Then 
it will start a thread that monitors the PVT buffer and fills it with fresh PVT 
data as the move progresses. While that internal CML thread is running, the main 
thread will wait for OUT1 to go low, meaning that the trajectory genertor is no 
longer running (the move is complete). 

The user should specify the positions (units are encoder counts) to traverse in 
the PVT stream using the position array shown below. The user should also specify 
the time it will take to travel to each position (units are milliseconds). The 
PvtConstAccelTrj class will calculate the velocity data for the PVT stream so that
acceleration will be continuous between waypoints, thereby reducing the risk of 
a following error. 

Make sure to travel to the first position of the PVT stream before beginning the 
PVT move or a following error will result. This example sets the last position 
of the PVT stream equal to the first position so that we automatically end up where
we started. 

Make sure to toggle digital input 1 to start the move. 

*/

#include <iostream>
#include "CML.h"

using std::cout;

// Comment this out to use EtherCAT
//#define USE_CAN

// Comment this out if not driving a stepper motor
//#define DRIVING_STEPPER

// Dual-axis application
#define AXIS_NUM 2

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

/* local data */
int32 canBPS = 1000000;             // CAN network bit rate
int16 canNodeID = 1;                // CANopen node ID

double positionsArr[407] = { -9558.1607,-9849.06,-11270.5,-14722.4,-19392.7,-24063,-27515,-28936.3,-29139.4107,-27633.1607,-26126.9107,-24620.6607,-23114.4107,-21608.1607,
-20101.9107,-18595.6607,-17089.4107,-15583.1607,-14076.9107,-12570.6607,-11064.4107,-9558.1607,-8051.9107,-6545.6607,-5039.4107,-3533.1607,-2026.9107,-520.6607,985.589275,
2491.839275,3998.089275,5504.339275,7010.589275,8516.839275,10023.08928,11529.33928,13035.58928,14541.83928,16048.08928,17554.33928,19060.58928,20566.83928,22073.08928,
23579.33928,25085.58928,26591.83928,28098.08928,29604.33928,29604.33928,29604.33928,29604.33928,29604.33928,29604.33928,29604.33928,29604.33928,29604.33928,29604.33928,
29604.33928,29604.33928,29604.33928,29604.33928,29604.33928,29604.33928,29604.33928,29604.33928,29604.33928,29604.33928,29604.33928,29604.33928,29604.33928,29604.33928,
29604.33928,29604.33928,29604.33928,29604.33928,29604.33928,29604.33928,29604.33928,29604.33928,29604.33928,29604.33928,29604.33928,29604.33928,29604.33928,29604.33928,
29604.33928,29604.33928,28098.08928,26591.83928,25085.58928,23579.33928,22073.08928,20566.83928,19060.58928,17554.33928,16048.08928,14541.83928,13035.58928,11529.33928,
10023.08928,8516.839275,7010.589275,5504.339275,3998.089275,2491.839275,985.589275,-520.6607,-2026.9107,-3533.1607,-5039.4107,-6545.6607,-8051.9107,-9558.1607,-11064.4107,
-12570.6607,-14076.9107,-15583.1607,-17089.4107,-18595.6607,-20101.9107,-21608.1607,-23114.4107,-24620.6607,-26126.9107,-27633.1607,-29139.4107,-29139.4107,-29139.4107,
-29139.4107,-29139.4107,-29139.4107,-29139.4107,-29139.4107,-29139.4107,-29139.4107,-29139.4107,-29139.4107,-29139.4107,-29139.4107,-29139.4107,-29139.4107,-29139.4107,
-29139.4107,-29139.4107,-29139.4107,-29139.4107,-29139.4107,-29139.4107,-29139.4107,-29139.4107,-29139.4107,-29139.4107,-29139.4107,-29139.4107,-29139.4107,-29139.4107,
-29139.4107,-29139.4107,-29139.4107,-29139.4107,-29139.4107,-29139.4107,-29139.4107,-29139.4107,-27633.1607,-26126.9107,-24620.6607,-23114.4107,-21608.1607,-20101.9107,
-18595.6607,-17089.4107,-15583.1607,-14076.9107,-12570.6607,-11064.4107,-9558.1607,-8051.9107,-6545.6607,-5039.4107,-3533.1607,-2026.9107,-520.6607,985.589275,2491.839275,
3998.089275,5504.339275,7010.589275,8516.839275,10023.08928,11529.33928,13035.58928,14541.83928,16048.08928,17554.33928,19060.58928,20566.83928,22073.08928,23579.33928,
25085.58928,26591.83928,28098.08928,28098.08928,28098.08928,28098.08928,28098.08928,28098.08928,28098.08928,28098.08928,28098.08928,28098.08928,28098.08928,28098.08928,
28098.08928,28098.08928,28098.08928,28098.08928,28098.08928,28098.08928,28098.08928,28098.08928,28098.08928,28098.08928,28098.08928,28098.08928,28098.08928,28098.08928,
28098.08928,28098.08928,28098.08928,28098.08928,28098.08928,28098.08928,28098.08928,28098.08928,28098.08928,28098.08928,28098.08928,28098.08928,26591.83928,25085.58928,
23579.33928,22073.08928,20566.83928,19060.58928,17554.33928,16048.08928,14541.83928,13035.58928,11529.33928,10023.08928,8516.839275,7010.589275,5504.339275,3998.089275,
2491.839275,985.589275,-520.6607,-2026.9107,-3533.1607,-5039.4107,-6545.6607,-8051.9107,-9558.1607,-11064.4107,-12570.6607,-14076.9107,-15583.1607,-17089.4107,-18595.6607,
-20101.9107,-21608.1607,-23114.4107,-24620.6607,-26126.9107,-27633.1607,-27633.1607,-27633.1607,-27633.1607,-27633.1607,-27633.1607,-27633.1607,-27633.1607,-27633.1607,
-27633.1607,-27633.1607,-27633.1607,-27633.1607,-27633.1607,-27633.1607,-27633.1607,-27633.1607,-27633.1607,-27633.1607,-27633.1607,-27633.1607,-27633.1607,-27633.1607,
-27633.1607,-27633.1607,-27633.1607,-27633.1607,-27633.1607,-27633.1607,-27633.1607,-27633.1607,-27633.1607,-27633.1607,-27633.1607,-27633.1607,-27633.1607,-27633.1607,
-26126.9107,-24620.6607,-23114.4107,-21608.1607,-20101.9107,-18595.6607,-17089.4107,-15583.1607,-14076.9107,-12570.6607,-11064.4107,-9558.1607,-8051.9107,-6545.6607,
-5039.4107,-3533.1607,-2026.9107,-520.6607,985.589275,2491.839275,3998.089275,5504.339275,7010.589275,8516.839275,10023.08928,11529.33928,13035.58928,14541.83928,16048.08928,
17554.33928,19060.58928,20566.83928,22073.08928,23579.33928,25085.58928,26591.83928,26591.83928,26591.83928,26591.83928,26591.83928,26591.83928,26591.83928,26591.83928,
26591.83928,26591.83928,26591.83928,26591.83928,26591.83928,26591.83928,26591.83928,26591.83928,26591.83928,26591.83928,26591.83928,26591.83928,26591.83928,26591.83928,
26591.83928,26591.83928,26591.83928,26591.83928,26591.83928,26591.83928,26591.83928,26591.83928,26591.83928,26591.83928,26591.83928,26591.83928,26591.83928,26591.83928,
25085.58928,23579.33928,22073.08928,20566.83928,19060.58928,17554.33928,16048.08928,14541.83928,13035.58928,11529.33928,10023.08928,8516.839275,7010.589275,5504.339275,
3998.089275,2491.839275,985.589275,-520.6607,-2026.9107,-3533.1607,-5039.4107,-6545.6607,-8051.9107,-9558.1607 };

// Load the PVT data into the PvtConstAccelTrj object.
// pvtConstTrjObj : a reference to an instance of the PvtConstAccelTrj class.
// isStartOfMove  : is this the start of the PVT move? If so, just load the
//                  first 32 PVT points into the pvtConstTrjObj. 
void loadPvtPoints(PvtConstAccelTrj& pvtConstTrjObj, bool isStartOfMove) 
{
	try 
	{
		uint8 timeBetweenPoints = 50; // units are milliseconds

		const Error* err = 0;
		vector<double> tempVec = { 0.0, 0.0 };
		vector<double>* tempVecPntr = &tempVec;

		vector<double> xPosVec;
		vector<double> yPosVec;

		vector<uint8> timeVec;
		timeVec.push_back(timeBetweenPoints); // 50ms
		
		int startingIndex = 32;
		int endingIndex = 407;

		// if this is the start of the move, load the first 32 points
		if (isStartOfMove) 
		{
			startingIndex = 0; 
			endingIndex = 32;
		}

		for (int i = startingIndex; i < endingIndex; i++) 
		{
			tempVec[0] = positionsArr[i];
			tempVec[1] = positionsArr[i]; // for now axis B will just mirror axis A behavior.

			err = pvtConstTrjObj.addPvtPoint(tempVecPntr, &timeVec[0]);
			showerr(err, "adding points to the PVT object");
		}
	}
	// catch any exception here
	catch (...) 
	{
		cout << "\nException occured in loadPvtPoints.\n";
		throw;
	}
}

// Define a class based on the TPDO base class. A Transmit PDO is 
// one that's transmitted by the CANopen node and received by the 
// master. This PDO will be used to send the digital output states 
// from the drive to the master (CML).
class TpdoDigitalOutputs : public TPDO
{

public:

	// Default constructor does nothing
	TpdoDigitalOutputs() {}

	// Called once at startup to map the PDO and configure CML to 
	// listen for it.
	const Error* Init(Amp& ampObj, int slot);

	// This function is called when the PDO is received
	virtual void Received(void);

	bool display = false;

	uint16 lastDigitalOutputValue = 0;

private:

	// These variables are used to map objects to the PDO.
	// Different mapping objects are used for different data sizes.
	Pmap16 digitalOutputs;
};

/**
 * Non-Fixed Transmit PDO handling.
 * Each axis is being configured to send out a PDO message
 * once every SYNC period.  This class handles receiving these
 * PDOs.
 *
 * @param ampObj The axis to map this TxPDO on.
 * @param slotNumber  The slot number used by this TxPDO.
 * @return NULL on success, or an error object on failure.
 */
const Error* TpdoDigitalOutputs::Init(Amp& ampObj, int slotNumber)
{
	const Error* err = NULL;

	// Initialize the transmit PDO with the CAN message ID if using a CAN network.
#ifdef USE_CAN
	err = TPDO::Init(0x280 + slotNumber * 0x100 + ampObj.GetNodeID());
#endif // USE_CAN

	// Init the variables using the CANopen object ID 
	if (!err) err = digitalOutputs.Init(0x2194, 0); //   16 bits
	// ---------
	//   32 bits
// Add the mapped variables to the PDO
	if (!err) err = AddVar(digitalOutputs);

	// Program this PDO in the amp, and enable it
	if (!err) err = ampObj.PdoSet(slotNumber, *this);

	return err;
}

/**
 * This function will be called by the high priority CANopen receive
 * thread when the PDO is received.
 *
 * By the time this function is called, the data from the two mapped objects will
 * have been parsed from the input message.  It can be accessed by the Pmap objects
 * that we created.
 *
 * Keep in mind that this function is called from the same thread that receives all
 * CANopen messages.  Keep any processing here short and don't try to do any SDO access.
 * Often it's best to simply post a semaphore here and have another thread handle the data.
 *
 */
void TpdoDigitalOutputs::Received(void)
{
	lastDigitalOutputValue = digitalOutputs.Read();
}

int main(void)
{
	// The libraries define one global object of type
	// CopleyMotionLibraries named cml.
	//
	// This object has a couple handy member functions
	// including this one which enables the generation of
	// a log file for debugging
	cml.SetDebugLevel(LOG_DEBUG);

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

	// Initialize the amplifiers using default settings
	Amp amp[AXIS_NUM];

	AmpSettings ampSettings;
	ampSettings.enableOnInit = false;

	// create TxPDO objects
	TpdoDigitalOutputs digitalOutputsTpdo;

	// Initializing axis A of a two-axis EtherCAT drive.
	printf("Initing Axis %d\n", 1);
	err = amp[0].Init(net, -1, ampSettings);
	showerr(err, "Initing Axis A");

	// initialize axis B of the two-axis EtherCAT drive.
	err = amp[1].InitSubAxis(amp[0], 2);
	showerr(err, "Initing Axis B of ME3");

	// pre-op the drive (only need to do it to axis a)
	err = amp[0].PreOpNode();
	showerr(err, "Preopping node");

	// initializing the non-fixed TxPDO
	err = digitalOutputsTpdo.Init(amp[0], 2); // use slot 2 or 3.
	showerr(err, "Initting non-fixed tpdo");

	// starting node
	err = amp[0].StartNode();
	showerr(err, "Starting node");

	// Create a linkage object holding these amps
	Linkage link;
	err = link.Init(AXIS_NUM, amp);
	showerr(err, "Linkage init");

	double pathMaxVel{ 2000000 };
	double pathMaxAccel{ 960000 };
	double pathMaxDecel{ 960000 };
	double pathMaxJerk{ 200000 };

	err = link.SetMoveLimits(pathMaxVel, pathMaxAccel, pathMaxDecel, pathMaxJerk);
	showerr(err, "Setting Linkage Move Limits");

	// 0x0d is the "write parameter" op-code. 
	// ASCII parameter 0x0078 is IN1 configuration. Set it to a value of 0x0028 for trajectory update on LO-HI transition. 
	byte input1ConfigTrjUpdateLoHi[5] = { 0x0d, 0x78, 0x00, 0x28, 0x00 };
	err = amp[0].Download(0x2000, 0, 5, input1ConfigTrjUpdateLoHi);
	showerr(err, "Setting IN1 config as trajectory update on lo-hi transition");

	// 0x0d is the "write parameter" op-code. 
	// ASCII parameter 0x0070 is OUT1 configuration. Set it to a value of 0x000300008000 for 
	// "Custom Trajectory Status," "Trajectory Generator Running," and "Output Active On." 
	byte out1ConfigTrjGenRunning[9] = { 0x0d, 0x70, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x80 };
	err = amp[0].Download(0x2000, 0, 9, out1ConfigTrjGenRunning);
	showerr(err, "Setting OUT1 config as custom trajectory status, trajectory generator running, output active on");

	// create an instance of the PvtConstAccelTrj class.
	PvtConstAccelTrj pvtConstTrjObj;

	// initialize the object with the number of dimensions in the trajectory.
	pvtConstTrjObj.Init(AXIS_NUM);
	showerr(err, "initializing the PvtConstAccelTrj object");

	// load the first 32 PVT points into the PVT object.
	loadPvtPoints(pvtConstTrjObj, true);

	// extract the starting point for the PVT stream.
	Point<AXIS_NUM> startingPoint;
	vector<list<double>>* posPntr = pvtConstTrjObj.getPositionsPntr();
	startingPoint[0] = (*posPntr)[0].front();
	startingPoint[1] = (*posPntr)[1].front();

	// NOTE: change this value to 21 if driving a non-stepper motor. 
	// Use 31 if driving a stepper motor.
	uint16 programmedDesiredState = 21;

#if defined (DRIVING_STEPPER)
	programmedDesiredState = 31;
#endif // DRIVING_STEPPER

	err = amp[0].sdo.Dnld16(0x2300, 0, programmedDesiredState);
	showerr(err, "Setting the desired state on axis A to programmed position mode");

	// NOTE: change this value to 21 if driving a non-stepper motor. 
	// Use 31 if driving a stepper motor.
	err = amp[0].sdo.Dnld16(0x2300 + 0x800, 0, programmedDesiredState);
	showerr(err, "Setting the desired state on axis B to programmed position mode");

	err = amp[0].sdo.Dnld32(0x607a, 0, (int32)startingPoint[0]);
	showerr(err, "Setting the target position for axis A");

	err = amp[0].sdo.Dnld32(0x607a + 0x800, 0, (int32)startingPoint[1]);
	showerr(err, "Setting the target position for axis B");

	// 0x0d is the "write parameter" op-code. 
	// ASCII parameter 0x00c8 is "Traj Config" axis A. Set it to a value of 0x0001 for Scurve mode. 
	byte setTrajConfigScurveAxisA[5] = { 0x0d, 0xc8, 0x00, 0x01, 0x00 };
	err = amp[0].Download(0x2000, 0, 5, setTrajConfigScurveAxisA);
	showerr(err, "Setting the trajectory config to Scurve mode on axis A");

	// 0x0d is the "write parameter" op-code. 
	// ASCII parameter 0x20c8 is "Traj Config" axis B. Set it to a value of 0x0001 for Scurve mode. 
	byte setTrajConfigScurveAxisB[5] = { 0x0d, 0xc8, 0x20, 0x01, 0x00 };
	err = amp[0].Download(0x2000, 0, 5, setTrajConfigScurveAxisB);
	showerr(err, "Setting the trajectory config to Scurve mode on axis B");

	// NOTE: Axes A & B will move at the same time to the starting position. 
	// 0x11 is the op-code for trajectory commands.
	// 0x3001 means Bit 12 (Axis A), Bit 13 (Axis B) and bit 0 (Trajectory Update Command).
	byte trajUpdate[5] = { 0x11, 0x01, 0x30 };
	err = amp[0].Download(0x2000, 0, 5, trajUpdate);
	showerr(err, "Moving to starting position");

	CML::Thread::sleep(1000); // wait one second for the move to start

	// wait for OUT1 to clear, meaning the move is finished.
	while (digitalOutputsTpdo.lastDigitalOutputValue & 1) {}

	// 0x0d is the "write parameter" op-code. 
	// ASCII parameter 0x00c8 is "Traj Config" axis A. Set it to a value of 0x0003 for PVT mode. 
	byte setTrajConfigPvtAxisA[5] = { 0x0d, 0xc8, 0x00, 0x03, 0x00 };
	err = amp[0].Download(0x2000, 0, 5, setTrajConfigPvtAxisA);
	showerr(err, "Setting the trajectory config to PVT mode on axis A");

	// 0x0d is the "write parameter" op-code. 
	// ASCII parameter 0x00c8 is "Traj Config" axis A. Set it to a value of 0x0003 for PVT mode. 
	byte setTrajConfigPvtAxisB[5] = { 0x0d, 0xc8, 0x20, 0x03, 0x00 };
	err = amp[0].Download(0x2000, 0, 5, setTrajConfigPvtAxisB);
	showerr(err, "Setting the trajectory config to PVT mode on axis B");

	// send the trajectory to the linkage.
	err = link.SendTrajectory(pvtConstTrjObj, false);
	showerr(err, "sending first 32 PVT points");

	// Make the PVT move five times in a row.
	for (int i = 0; i < 5; i++) {

		// waiting for move to start
		err = amp[0].WaitInputHigh(1);
		showerr(err, "waiting for IN1 to go high");

		// load the rest of the points into the PVT object
		loadPvtPoints(pvtConstTrjObj, false);

		// send the PVT object to the linkage for streaming. CML will 
		// handle the PVT buffer management in a non-blocking thread.
		err = link.SendTrajectory(pvtConstTrjObj, false);
		showerr(err, "sending rest of PVT points");

		// wait for the move to finish (OUT1 is clear)
		while(digitalOutputsTpdo.lastDigitalOutputValue & 1){ }

		// load the first 32 PVT points into the PVT object for the next iteration. 
		loadPvtPoints(pvtConstTrjObj, true);

		// load the first 32 PVT points into the drive
		err = link.SendTrajectory(pvtConstTrjObj, false);
		showerr(err, "sending first 32 PVT points");
	}

	uint16 canOpenDesiredState = 30;

#if defined (DRIVING_STEPPER)
	canOpenDesiredState = 40;
#endif // DRIVING_STEPPER

	// Configure Axis A for CANopen over EtherCAT (CoE) Stepper mode (40).
	// NOTE: change this value to 30 if driving a non-stepper motor. 
	// Use 40 if driving a stepper motor.
	err = amp[0].sdo.Dnld16(0x2300, 0, canOpenDesiredState);
	showerr(err, "Setting the desired state on axis A back to CANopen mode");

	// Configure Axis A for CANopen over EtherCAT (CoE) Stepper mode (40).
	// NOTE: change this value to 30 if driving a non-stepper motor. 
	// Use 40 if driving a stepper motor.
	err = amp[0].sdo.Dnld16(0x2300 + 0x800, 0, canOpenDesiredState);
	showerr(err, "Setting the desired state on axis B back to CANopen mode");

	printf("Move finished\n");
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