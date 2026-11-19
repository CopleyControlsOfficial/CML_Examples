# Copley Motion Library (CML)

CML is a C++ library for building motion control applications that run Copley drives over EtherCAT and CANopen, from one axis to a coordinated robot.

CML runs on a PC or embedded computer as the network master. It brings up the network, configures each drive, and commands motion: single-axis moves, coordinated multi-axis moves, and cyclic synchronous streaming with distributed-clock synchronization. On top of that it adds robot kinematics, Cartesian path planning and keep-out zones, so an application can command a robot by tool pose instead of joint by joint.

## Key features

- **One API for EtherCAT and CANopen.** The same application code runs on either network.
- **Coordinated multi-axis motion.** Group up to 32 axes into a linkage that starts, moves and stops together.
- **All the CiA 402 motion modes Copley drives use.** Profile position, velocity and torque, homing, PVT and PT streaming, and cyclic synchronous position, velocity and torque.
- **Synchronized cycles.** Distributed clocks on EtherCAT, SYNC on CANopen, and a built-in cycle timing and jitter report.
- **Robot kinematics.** SCARA, delta, 6-axis articulated (with URDF import), wafer handlers and Stewart platforms, with Cartesian path planning and keep-out zones.
- **PLCopen Motion** PLCopen Motion Control function blocks for single axes and axis groups, and a built-in PLC engine that runs Structured Text programs.
- **Drive configuration in code.** Load configuration files saved from CME, read and write drive parameters, and update drive firmware.
- **I/O and third-party devices.** Copley and CiA 401 I/O modules, and other devices configured from their ESI files.

## How CML fits in a system

- Your application, written in C++, calls CML.
- CML runs as the EtherCAT or CANopen master on Windows or Linux, including PREEMPT_RT real-time kernels.
- It uses a standard Ethernet port or a CAN card. No special master hardware is needed.
- It commands Copley servo and stepper drives, I/O modules, and other EtherCAT or CANopen devices.
- Drives are set up and tuned first in CME.

## Specifications

- **Delivery:** C++14 (ANSI) source code. Build it into your application as a static library, DLL or shared object.
- **Operating systems:** Windows. Linux, including PREEMPT_RT real-time kernels. x86-64 and ARM64 processors, including NVIDIA Jetson.
- **EtherCAT master:** CoE with SDO and fixed or custom PDO mapping. Distributed clocks. Logical addressing (FMMU). Third-party devices configured from their ESI files (optional component).
- **Ethernet over EtherCAT** CME reaches drives through the running master. On Windows this uses the free TAP-Windows6 driver, installed separately.
- **EtherCAT network access:** any standard Ethernet port. Windows: EtherCAT over UDP. Linux: raw Ethernet socket.
- **Cycle period:** set by the application. Sub-millisecond on real-time Linux (250 µs, 4 kHz, demonstrated); milliseconds on Windows.
- **CANopen master:** CiA 301 with SDO, PDO, SYNC, heartbeat and node guarding, and LSS. CiA 402 drives and CiA 401 I/O modules.
- **CAN interfaces:** Copley CAN cards, IXXAT, Kvaser, National Instruments, PEAK-System, Siemens, Linux SocketCAN.
- **Motion modes:** profile position, velocity and torque; homing; interpolated PVT and PT; cyclic synchronous position, velocity, torque, and torque with commutation angle, on EtherCAT and CANopen.
- **Multi-axis motion:** linkages of up to 32 axes (configurable), with coordinated moves, streamed user trajectories and a coordinated halt.
- **Trajectories:** S-curve and trapezoidal profiles, PVT streaming, and checks of a planned move against velocity, acceleration and jerk limits before it runs.
- **Kinematics:** SCARA, delta, 6-axis articulated (with URDF import), wafer handling robots and Stewart platforms. Cartesian lines and blended paths, keep-out zones and self-collision checks.
- **PLCopen Motion** function blocks for single axes, axis groups, homing, camming, gearing and path moves, and a PLC engine that compiles and runs Structured Text, with program variables linked to drive PDOs.
- **Drives and devices:** Copley Accelnet, Xenus, Stepnet, Argus and R-Series servo and stepper drives on CANopen or EtherCAT. Other devices through the generic node interface.
- **Drive configuration:** load CME .ccx and .ccd files, read and write any drive parameter, save to flash, update firmware. Map your own objects into the PDOs CML leaves free.
- **Network configuration file:** open a whole network from one file that names each node's identity, distributed-clock settings, startup SDOs and axes, with each node checked against the file before it is configured.
- **Diagnostics:** an error code from every call, adjustable logging, and an EtherCAT cycle timing and jitter report.
- **Examples included:** single- and two-axis moves, gantry, each cyclic mode, robots (SCARA, delta, 6-axis, hexapod, wafer), and I/O.
- **Licensing:** one-time license fee, with no recurring charges. Includes the source code, examples and reference manual. Contact Copley Controls sales for pricing.
