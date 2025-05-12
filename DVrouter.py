####################################################
# DVrouter.py
# Name: Distance Vector Router Implementation
# HUID: 
#####################################################

from router import Router
from packet import Packet
import json

class DVrouter(Router):
    """Distance vector routing protocol implementation.

    Add your own class fields and initialization code (e.g. to create forwarding table
    data structures). See the `Router` base class for docstrings of the methods to
    override.
    """

    def __init__(self, addr, heartbeat_time):
        Router.__init__(self, addr)  # Initialize base class - DO NOT REMOVE
        self.heartbeat_time = heartbeat_time
        self.last_time = 0
        
        # Distance vector table (destination -> (cost, next_hop_port))
        self.dv = {addr: (0, None)}  # Own address has cost 0
        
        # Keep track of neighbors and their addresses and costs
        self.neighbors = {}  # port -> (addr, cost)
        
        # Store the latest distance vectors from each neighbor
        self.neighbor_dvs = {}  # neighbor_addr -> {dst: cost}
        
        # Define infinity for "count to infinity" problem
        self.INFINITY = 16

    def handle_packet(self, port, packet):
        """Process incoming packet."""
        if packet.is_traceroute:
            # Handle normal data packet (traceroute)
            dst = packet.dst_addr
            if dst in self.dv and self.dv[dst][0] < self.INFINITY:
                next_hop_port = self.dv[dst][1]
                if next_hop_port is not None:
                    self.send(next_hop_port, packet)
        else:
            # Handle routing packet (DV update from neighbor)
            try:
                # Extract neighbor's distance vector
                neighbor_addr = packet.src_addr
                neighbor_dv = json.loads(packet.content)
                
                updated = False
                # Check if the received DV is different from what we have
                if neighbor_addr not in self.neighbor_dvs or self.neighbor_dvs[neighbor_addr] != neighbor_dv:
                    # Store the new distance vector
                    self.neighbor_dvs[neighbor_addr] = neighbor_dv
                    updated = True
                
                if updated:
                    # Recalculate our distance vector using Bellman-Ford
                    self._update_distance_vector()
                    # Broadcast our updated DV to all neighbors
                    self._broadcast_dv()
            except json.JSONDecodeError:
                # Ignore malformed packets
                pass

    def handle_new_link(self, port, endpoint, cost):
        """Handle new link."""
        # Add new neighbor
        self.neighbors[port] = (endpoint, cost)
        
        # Update our distance vector with the direct path to this neighbor
        self.dv[endpoint] = (cost, port)
        
        # Recalculate our distance vector
        self._update_distance_vector()
        
        # Broadcast our updated DV to all neighbors including the new one
        self._broadcast_dv()

    def handle_remove_link(self, port):
        """Handle removed link."""
        if port in self.neighbors:
            neighbor_addr = self.neighbors[port][0]
            
            # Remove the neighbor
            del self.neighbors[port]
            
            # Remove the neighbor's DV
            if neighbor_addr in self.neighbor_dvs:
                del self.neighbor_dvs[neighbor_addr]
            
            # Recalculate our distance vector
            self._update_distance_vector()
            
            # Broadcast our updated DV to remaining neighbors
            self._broadcast_dv()

    def handle_time(self, time_ms):
        """Handle current time."""
        if time_ms - self.last_time >= self.heartbeat_time:
            self.last_time = time_ms
            # Broadcast our distance vector periodically
            self._broadcast_dv()

    def _update_distance_vector(self):
        """Update the distance vector using Bellman-Ford algorithm."""
        # Reset our DV to only include direct connections and ourselves
        updated_dv = {self.addr: (0, None)}
        
        # Add direct connections to neighbors
        for port, (neighbor_addr, cost) in self.neighbors.items():
            updated_dv[neighbor_addr] = (cost, port)
        
        # Apply Bellman-Ford algorithm
        for neighbor_addr, neighbor_dv in self.neighbor_dvs.items():
            # Find the port and cost to this neighbor
            neighbor_port = None
            neighbor_cost = self.INFINITY
            
            for port, (addr, cost) in self.neighbors.items():
                if addr == neighbor_addr:
                    neighbor_port = port
                    neighbor_cost = cost
                    break
            
            if neighbor_port is None:
                # Neighbor no longer exists
                continue
            
            # Calculate routes through this neighbor
            for dst, cost in neighbor_dv.items():
                if dst == self.addr:
                    # Skip routes to ourselves
                    continue
                
                # Calculate new cost through this neighbor
                new_cost = neighbor_cost + cost
                
                # Apply "split horizon with poisoned reverse" to avoid count-to-infinity
                if new_cost >= self.INFINITY:
                    new_cost = self.INFINITY
                
                # Update our DV if we found a better path or this is a new destination
                if dst not in updated_dv or new_cost < updated_dv[dst][0]:
                    updated_dv[dst] = (new_cost, neighbor_port)
        
        # Update our distance vector with the new calculated values
        self.dv = updated_dv

    def _broadcast_dv(self):
        """Broadcast our distance vector to all neighbors."""
        # Prepare our distance vector for sending (exclude next hop information)
        dv_to_send = {}
        for dst, (cost, _) in self.dv.items():
            dv_to_send[dst] = cost
        
        # Create a packet for each neighbor with appropriate split horizon
        for port, (neighbor_addr, _) in self.neighbors.items():
            # Apply split horizon with poisoned reverse
            poisoned_dv = dv_to_send.copy()
            
            # Apply poisoned reverse: if we route to a destination through this neighbor,
            # we advertise an infinite cost to that destination to this neighbor
            for dst, (cost, next_hop_port) in self.dv.items():
                if next_hop_port == port:
                    poisoned_dv[dst] = self.INFINITY
            
            # Create and send the routing packet
            packet = Packet(Packet.ROUTING, self.addr, neighbor_addr, json.dumps(poisoned_dv))
            self.send(port, packet)

    def __repr__(self):
        """Representation for debugging in the network visualizer."""
        routes = {}
        for dst, (cost, port) in self.dv.items():
            if cost < self.INFINITY:
                routes[dst] = (cost, port)
        
        return f"DVrouter(addr={self.addr}, routes={routes})"
