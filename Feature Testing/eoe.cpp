/** \file

CML example: Ethernet over EtherCAT (EoE)

Opens an EtherCAT network, initializes every Copley drive on it, and joins
each drive that declares EoE to a virtual network adapter on this PC.  While
it runs, programs on the PC reach the drives through that adapter - CME over
UDP, for example - while CML runs the network.

   eoe <EtherCAT NIC> [PC address] [subnet mask] [-ip <first drive address>]
       [-maconly] [all]

Nothing is set in the drive's own parameters: its IP address (0x11F) can stay
0.  Each drive is given a virtual MAC and an IP address over EoE (TwinCAT's
"IP port" setting); neither is stored in the drive.  The addresses start at
the PC address + 10 (192.168.100.11, .12, ... in network order) unless -ip
gives the first one.  CME's UDP scan reports each drive's current IP address
and CME then sends its commands there, so the drive needs one on the
adapter's subnet.

-maconly sends only the MAC (TwinCAT's "switch port" setting).  A drive then
keeps the IP address it already has, which is 0 unless 0x11F is set.

The PC's adapter defaults to 192.168.100.1 with mask 255.255.255.0.

The switch passes only ARP and IPv4 that is not multicast from the PC to the
drives, which keeps the PC's own IPv6 and multicast traffic out of the drives'
mailboxes.  CME's scan is a broadcast, so it passes.  "all" passes everything.

Linux: run as root.  The adapter is a TAP interface named cmleoe0, created
and given the PC address here.

Windows: install the TAP-Windows6 driver, which adds one adapter.  Nothing else
is set up: the driver hands Windows the PC address when Windows asks for one by
DHCP, as it does for an adapter left on automatic.  An adapter given a static
address in its own settings keeps it, and the drives' addresses are taken from
that instead.  An adapter in a Windows network bridge has no address of its
own; give the bridge's address and mask here.

The EoE adapter's subnet must not hold the EtherCAT adapter's address.

Press Enter to stop.
*/

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "CML.h"
#include "ecat/eoe_tap.h"

#if defined( WIN32 ) || defined( _WIN32 )
#include "ecat/ecat_winudp.h"
#else
#include "ecat/ecat_linux.h"
#endif

CML_NAMESPACE_USE();

#define MAX_DRIVES  32

static void showerr( const Error *err, const char *what )
{
   if( !err ) return;
   printf( "Error %s: %s\n", what, err->toString() );
   exit( 1 );
}

static uint32 parseIp( const char *text, const char *what )
{
   uint32 a = 0;
   if( !EoeIpParams::ParseIp( text, a ) )
   {
      printf( "%s '%s' is not an IPv4 address\n", what, text );
      exit( 1 );
   }
   return a;
}

int main( int argc, char **argv )
{
   cml.SetDebugLevel( LOG_ERRORS );

   EoeSwitchSettings settings;
   const char *pos[3] = { "eth0", "192.168.100.1", "255.255.255.0" };
   const char *ipArg = 0;
   bool macOnly = false;
   int npos = 0;
   for( int i = 1; i < argc; i++ )
   {
      if( !strcmp( argv[i], "all" ) ) settings.forwardAll = true;
      else if( !strcmp( argv[i], "-maconly" ) ) macOnly = true;
      else if( !strcmp( argv[i], "-ip" ) && i + 1 < argc ) ipArg = argv[++i];
      else if( npos < 3 && argv[i][0] != '-' ) pos[npos++] = argv[i];
      else
      {
         printf( "Usage: eoe <EtherCAT NIC> [PC address] [subnet mask] [-ip <first drive address>] [-maconly] [all]\n" );
         return 1;
      }
   }

   const char *nic = pos[0];
#if defined( WIN32 ) || defined( _WIN32 )
   WinUdpEcatHardware hw( nic );
#else
   LinuxEcatHardware hw( nic );
#endif
   uint32 pcIp = parseIp( pos[1], "PC address" );
   uint32 mask = parseIp( pos[2], "Subnet mask" );
   char buff[20], mbuff[20];

   // The PC's adapter first: the drives' addresses depend on its subnet.
   EoeTap tap;
   showerr( tap.SetAddress( pcIp, mask ), "checking the PC address and mask" );
   showerr( tap.Open(), "opening the PC's EoE adapter" );
   uint32 tapIp = 0, tapMask = 0;
   tap.GetAddress( tapIp, tapMask );
#if defined( WIN32 ) || defined( _WIN32 )
   // Windows asks for the address by DHCP once the adapter is connected, and
   // shows 169.254.x.x until it has it.  A static address shows at once.
   printf( "Waiting for Windows to configure the adapter '%s'...\n", tap.GetName() );
   for( int ms = 0; ( ms < 20000 && ( tapIp >> 16 ) == 0xA9FE ) || ( ms < 5000 && !tapIp ); ms += 250 )
   {
      Thread::sleep( 250 );
      tap.GetAddress( tapIp, tapMask );
   }
   if( ( tapIp >> 16 ) == 0xA9FE )
   {
      EoeIpParams::FormatIp( tapIp, buff, sizeof(buff) );
      printf( "The adapter '%s' did not get an address (it has %s).  In Network\n"
              "Connections, open its IPv4 properties and choose \"Obtain an IP address\n"
              "automatically\", or enter a static address there.\n",
              tap.GetName(), buff );
      return 1;
   }
   // No IPv4 at all: the adapter is a member of a Windows network bridge, and the
   // address is on the bridge.  The PC address given here must be the bridge's.
   if( !tapIp )
   {
      EoeIpParams::FormatIp( pcIp, buff, sizeof(buff) );
      EoeIpParams::FormatIp( mask, mbuff, sizeof(mbuff) );
      printf( "The adapter has no IPv4 address of its own (bridged?).  Using %s mask %s,\n"
              "which must be the Network Bridge's address.\n", buff, mbuff );
   }
   else if( tapIp != pcIp || tapMask != mask )
   {
      EoeIpParams::FormatIp( tapIp, buff, sizeof(buff) );
      EoeIpParams::FormatIp( tapMask, mbuff, sizeof(mbuff) );
      printf( "The adapter has a static address, %s mask %s; using it.\n", buff, mbuff );
   }
#endif
   if( tapIp ) { pcIp = tapIp; mask = tapMask; }
   uint32 firstIp = ipArg ? parseIp( ipArg, "First drive address" ) : pcIp + 10;

   // CML's EtherCAT adapter on the same subnet would let Windows send
   // EtherCAT or CME traffic out of the wrong adapter.
   uint32 nicIp = 0;
   if( EoeIpParams::ParseIp( nic, nicIp ) && ( nicIp & mask ) == ( pcIp & mask ) )
   {
      EoeIpParams::FormatIp( pcIp, buff, sizeof(buff) );
      printf( "The EtherCAT adapter %s is on the EoE adapter's subnet (%s).  Give the EoE\n"
              "adapter an address on another subnet, such as 192.168.100.1.\n", nic, buff );
      return 1;
   }

   EtherCAT net;
   showerr( net.Open( hw ), "opening the EtherCAT network" );

   int count = net.getNodeCount();
   if( count > MAX_DRIVES ) count = MAX_DRIVES;
   printf( "%d node(s) found on %s\n", count, nic );

   Amp *amp = new Amp[ MAX_DRIVES ];
   for( int i = 0; i < count; i++ )
   {
      char what[64];
      snprintf( what, sizeof(what), "initializing the drive at position %d", i + 1 );
      showerr( amp[i].Init( net, (int16)( -( i + 1 ) ) ), what );
   }

   EoeSwitch sw;
   showerr( sw.Start( net, tap, settings ), "starting the EoE switch" );

   EoeIpParams::FormatIp( pcIp, buff, sizeof(buff) );
   EoeIpParams::FormatIp( mask, mbuff, sizeof(mbuff) );
   printf( "PC adapter %s: %s mask %s\n", tap.GetName(), buff, mbuff );

   int added = 0;
   for( int i = 0; i < count; i++ )
   {
      if( !net.EoeSupported( &amp[i] ) )
      {
         printf( "  drive %d: no EoE in its EEPROM, skipped\n", i + 1 );
         continue;
      }
      EoeIpParams ip;
      if( !macOnly )
      {
         uint32 a = firstIp + (uint32)added;
         // On the PC's subnet, and neither its network nor its broadcast address.
         if( ( a & mask ) != ( pcIp & mask ) || !( a & ~mask ) || ( a | mask ) == 0xFFFFFFFF || a == pcIp )
         {
            EoeIpParams::FormatIp( a, buff, sizeof(buff) );
            printf( "  drive %d: %s is not a usable address on the PC's subnet, skipped\n", i + 1, buff );
            continue;
         }
         ip.SetAddress( a, mask, pcIp );
      }
      const Error *err = sw.AddNode( amp[i], ip );
      if( macOnly ) snprintf( buff, sizeof(buff), "MAC only" );
      else EoeIpParams::FormatIp( ip.ip, buff, sizeof(buff) );
      if( err )
         printf( "  drive %d: not added: %s\n", i + 1, err->toString() );
      else
      {
         uint8 mac[6];
         uint16 at = 0;
         net.GetNodeAddress( &amp[i], at );
         EoeSwitch::DefaultMac( at, mac );
         printf( "  drive %d: %s, MAC %02X:%02X:%02X:%02X:%02X:%02X\n", i + 1, buff,
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5] );
         added++;
      }
   }

   printf( "\nPress Enter to stop.\n" );
   getchar();

   EoeSwitch::Stats st = sw.GetStats();
   printf( "Frames to drives %u, from drives %u, filtered %u, unknown destination %u, errors %u\n",
           (unsigned)st.hostToNode, (unsigned)st.nodeToHost, (unsigned)st.filtered,
           (unsigned)st.unknownDest, (unsigned)st.sendErrors );

   // The switch first: it uses the drives and the network.
   sw.Stop();
   delete[] amp;
   net.Close();
   return 0;
}
