// MIT License
//
// Copyright (c) 2016-2022 Mark Qvist / unsigned.io
// C# Port (c) 2023 Michael Faragher / betweentheborders.com
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// This is considered pre-alpha of a minimum viable product. No warranties are provided and compatibility is not assured.

namespace RNS
{
    public class Transport
    {
        const byte BROADCAST = 0x00;
        const byte TRANSPORT = 0x01;
        const byte RELAY = 0x02;
        const byte TUNNEL = 0x03;
        byte[] types = {BROADCAST, TRANSPORT, RELAY, TUNNEL};
        static public Dictionary<byte,string> PropagationType = new Dictionary<byte,string>() { { 0x00, "Broadcast" }, { 0x01, "Transport" }, { 0x02,"Relay"},{ 0x03, "Tunnel" } };

        static public Dictionary<byte, string> DestinationType = new Dictionary<byte, string>() { {0x00,"Single" },{0x01,"Group" },{0x02,"Plain" },{0x03,"link" } };

        static public Dictionary<byte, string> PacketType = new Dictionary<byte, string>() { { 0x00, "Data" }, { 0x01, "announce" }, { 0x02, "Link Request" }, { 0x03, "Proof" } };

        const byte REACHABILITY_UNREACHABLE = 0x00;
        const byte REACHABILITY_DIRECT = 0x01;
        const byte REACHABILITY_TRANSPORT = 0x02;

    string APP_NAME = "rnstransport";

    int PATHFINDER_M = 128;     //Maximum amount of hops that Reticulum will transport a packet.


        int PATHFINDER_R = 1;        // Retransmit retries
        int PATHFINDER_G = 5;         // Retry grace period
        float PATHFINDER_RW = 0.5f;      // Random window for announce rebroadcast
        int PATHFINDER_E = 60 * 60 * 24 * 7; // Path expiration of one week
        int AP_PATH_TIME = 60 * 60 * 24;   // Path expiration of one day for Access Point paths
        int ROAMING_PATH_TIME = 60 * 60 * 6;  // Path expiration of 6 hours for Roaming paths

        // TODO: Calculate an optimal number for this in
        // various situations
        int LOCAL_REBROADCASTS_MAX = 2;          // How many local rebroadcasts of an announce is allowed

        int PATH_REQUEST_TIMEOUT = 15;           // Default timuout for client path requests in seconds
        float PATH_REQUEST_GRACE = 0.35f;         // Grace time before a path announcement is made, allows directly reachable peers to respond first
        int PATH_REQUEST_RW = 2;            // Path request random window
        int PATH_REQUEST_MI = 5;            // Minimum interval in seconds for automated path requests

    float LINK_TIMEOUT = RNS.Link.STALE_TIME * 1.25f;
        int REVERSE_TIMEOUT = 30 * 60;        // Reverse table entries are removed after 30 minutes
        int DESTINATION_TIMEOUT = 60 * 60 * 24 * 7;   // Destination table entries are removed if unused for one week
        int MAX_RECEIPTS = 1024;         // Maximum number of receipts to keep track of
        int MAX_RATE_TIMESTAMPS = 16;           // Maximum number of announce timestamps to keep per destination

        List<Interface> interfaces = new List<Interface>();           // All active interfaces
//    destinations         = []           # All active destinations
//    pending_links        = []           # Links that are being established
//    active_links         = []           # Links that are active
//    packet_hashlist      = []           # A list of packet hashes for duplicate detection
//    receipts             = []           # Receipts of all outgoing packets for proof processing

//    # TODO: "destination_table" should really be renamed to "path_table"
//    # Notes on memory usage: 1 megabyte of memory can store approximately
//    # 55.100 path table entries or approximately 22.300 link table entries.

//    announce_table       = { }           # A table for storing announces currently waiting to be retransmitted
//    destination_table    = {}           # A lookup table containing the next hop to a given destination
//    reverse_table        = {}           # A lookup table for storing packet hashes used to return proofs and replies
//    link_table           = {}           # A lookup table containing hops for links
//    held_announces       = {}           # A table containing temporarily held announce-table entries
//    announce_handlers    = []           # A table storing externally registered announce handlers
//    tunnels              = {}           # A table storing tunnels to other transport instances
//    announce_rate_table  = {}           # A table for keeping track of announce rates
//    path_requests        = {}           # A table for storing path request timestamps
    
//    discovery_path_requests  = {}       # A table for keeping track of path requests on behalf of other nodes
//    discovery_pr_tags        = []       # A table for keeping track of tagged path requests
//    max_pr_tags              = 32000    # Maximum amount of unique path request tags to remember

//    # Transport control destinations are used
//    # for control purposes like path requests
//    control_destinations = []
//        control_hashes       = []

//# Interfaces for communicating with
//# local clients connected to a shared
//# Reticulum instance
//        local_client_interfaces = []

//        local_client_rssi_cache    = []
//        local_client_snr_cache     = []
//        LOCAL_CLIENT_CACHE_MAXSIZE = 512

//    pending_local_path_requests = {}

//    jobs_locked = False
//    jobs_running = False
//    job_interval = 0.250
//    links_last_checked       = 0.0
//    links_check_interval     = 1.0
//    receipts_last_checked    = 0.0
//    receipts_check_interval  = 1.0
//    announces_last_checked   = 0.0
//    announces_check_interval = 1.0
//    hashlist_maxsize         = 1000000
//    tables_last_culled       = 0.0
//    tables_cull_interval     = 5.0

//    identity = None
    }
}