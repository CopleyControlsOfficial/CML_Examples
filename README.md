# CML_Examples
Copley Motion Library (C++) Examples

What is CML?
-	CML is the Copley Motion Library. 
	It is written in C++ and is proprietary source code. 
	To purchase CML, please contact Copley Controls Corporation. 
	There are a variety of licensing options available. 
	CML is compatible with Windows, Linux, and QNX operating systems. 

CML Features:
-	CML implements the CANopen protocol for motion control over CAN or EtherCAT networks with Copley products. 
	The applications are limitless as CML supports the maximum number of nodes per network as defined by the protocol standard: 
	- 127 for CAN networks
	- 65,535 for EtherCAT networks
-	Real-time messaging through synchronous PDO’s and asynchronous SDO’s.
-	Linkage moves up to 32 axes of coordinated motion.
-	Control of any analog or digital I/O module compliant to the DS401 standard (Wago I/O, Copley I/O, Beckhoff EK1100, etc.).
-	Control of any EtherCAT device, even simple ESC devices that do not support mailbox protocols (CoE).
-	PVT streaming capabilities with built-in velocity calculation for smooth motion profiles.
-	Seamless integration into the Robot Operating System (ROS2) framework.

Important Settings:
-	If using CML to command an EtherCAT network on a non real-time operating system for an extended duration, the PDO update
  	rate may need to be adjusted to avoid generating EtherCAT message timeout errors. To compensate for the performance of the
 	operating system, simply increase the cyclePeriod property of the EtherCatSettings class. This property is the PDO update
 	rate in units of milliseconds. When increasing this property, it is also recommended to increase the guardTime property of
 	the AmpSettings class to avoid generating any node guarding errors. 
