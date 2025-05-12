####################################################
# LSrouter.py
# Name: Link State Router Implementation
# HUID: 
#####################################################

from router import Router
from packet import Packet
import json
import heapq


class LSrouter(Router):
    """Link state routing protocol implementation.

    Add your own class fields and initialization code (e.g. to create forwarding table
    data structures). See the `Router` base class for docstrings of the methods to
    override.
    """

    def __init__(self, addr, heartbeat_time):
        Router.__init__(self, addr)  # Initialize base class - DO NOT REMOVE
        self.heartbeat_time = heartbeat_time
        self.last_time = 0
        
        # Link-state database: {router_addr: {neighbor_addr: cost}}
        self.ls_db = {addr: {}}
        
        # Forwarding table: {dst: next_hop_port}
        self.forwarding_table = {}
        
        # Sequence numbers for each router's LSP
        # Used to distinguish between new and old updates
        self.sequence_numbers = {addr: 0}
        
        # Keep track of neighbors and their costs
        self.neighbors = {}  # port -> (addr, cost)
        
        # Keep track of which LSPs we have already forwarded
        # {router_addr: highest_seq_num_forwarded}
        self.forwarded_lsps = {}

    def handle_packet(self, port, packet):
        """Process incoming packet."""
        if packet.is_traceroute:
            # Handle normal data packet (traceroute)
            dst = packet.dst_addr
            if dst in self.forwarding_table:
                next_hop_port = self.forwarding_table[dst]
                self.send(next_hop_port, packet)
        else:
            # Handle routing packet (LSP)
            try:
                lsp_data = json.loads(packet.content)
                
                # Extract LSP information
                router_addr = lsp_data['router_addr']
                links = lsp_data['links']
                seq_num = lsp_data['seq_num']
                
                # Check if this is a new LSP (higher sequence number)
                is_new = False
                if router_addr not in self.sequence_numbers:
                    is_new = True
                elif seq_num > self.sequence_numbers[router_addr]:
                    is_new = True
                
                if is_new:
                    # Update sequence number
                    self.sequence_numbers[router_addr] = seq_num
                    
                    # Update link-state database
                    self.ls_db[router_addr] = links
                    
                    # Forward LSP to all neighbors except the one we received it from
                    self._flood_lsp(packet, port)
                    
                    # Recompute shortest paths
                    self._compute_shortest_paths()
            except (json.JSONDecodeError, KeyError):
                # Ignore malformed packets
                pass

    def handle_new_link(self, port, endpoint, cost):
        """Handle new link."""
        # Add new neighbor
        self.neighbors[port] = (endpoint, cost)
        
        # Update our link-state information
        self.ls_db[self.addr][endpoint] = cost
        
        # Increment our sequence number
        if self.addr in self.sequence_numbers:
            self.sequence_numbers[self.addr] += 1
        else:
            self.sequence_numbers[self.addr] = 1
        
        # Send our updated link-state information
        self._send_lsp()
        
        # Recompute shortest paths
        self._compute_shortest_paths()

    def handle_remove_link(self, port):
        """Handle removed link."""
        if port in self.neighbors:
            neighbor_addr = self.neighbors[port][0]
            
            # Remove neighbor
            del self.neighbors[port]
            
            # Update our link-state information
            if neighbor_addr in self.ls_db[self.addr]:
                del self.ls_db[self.addr][neighbor_addr]
            
            # Increment our sequence number
            self.sequence_numbers[self.addr] += 1
            
            # Send our updated link-state information
            self._send_lsp()
            
            # Recompute shortest paths
            self._compute_shortest_paths()

    def handle_time(self, time_ms):
        """Handle current time."""
        if time_ms - self.last_time >= self.heartbeat_time:
            self.last_time = time_ms
            
            # Periodically send our link-state information
            self._send_lsp()

    def _send_lsp(self):
        """Send Link State Packet (LSP) to all neighbors."""
        # Prepare link-state information
        lsp_data = {
            'router_addr': self.addr,
            'links': self.ls_db[self.addr],
            'seq_num': self.sequence_numbers[self.addr]
        }
        
        # Create and send LSP to all neighbors
        for port in self.neighbors:
            neighbor_addr = self.neighbors[port][0]
            packet = Packet(Packet.ROUTING, self.addr, neighbor_addr, json.dumps(lsp_data))
            self.send(port, packet)

    def _flood_lsp(self, packet, ingress_port):
        """Flood Link State Packet (LSP) to all neighbors except ingress port."""
        # Forward the packet to all neighbors except the one we received it from
        for port in self.neighbors:
            if port != ingress_port:
                neighbor_addr = self.neighbors[port][0]
                # Create a new packet with the neighbor as the destination
                new_packet = Packet(Packet.ROUTING, self.addr, neighbor_addr, packet.content)
                self.send(port, new_packet)

    def _compute_shortest_paths(self):
        """Compute shortest paths using Dijkstra's algorithm."""
        # Build graph from link-state database
        graph = {}
        for router, links in self.ls_db.items():
            if router not in graph:
                graph[router] = {}
            for neighbor, cost in links.items():
                if neighbor not in graph:
                    graph[neighbor] = {}
                graph[router][neighbor] = cost
        
        # Run Dijkstra's algorithm
        distances = {}  # Final shortest distances
        predecessors = {}  # Predecessor nodes for each destination
        pq = [(0, self.addr, None)]  # Priority queue: (distance, node, predecessor)
        visited = set()
        
        while pq:
            dist, current, pred = heapq.heappop(pq)
            
            if current in visited:
                continue
            
            visited.add(current)
            distances[current] = dist
            predecessors[current] = pred
            
            # Only continue if we know this node's neighbors
            if current in graph:
                for neighbor, cost in graph[current].items():
                    if neighbor not in visited:
                        heapq.heappush(pq, (dist + cost, neighbor, current))
        
        # Build forwarding table from shortest paths
        self.forwarding_table = {}
        
        # For each destination
        for dst in distances:
            if dst == self.addr:
                continue  # Skip self
            
            # Find the first hop on the path to destination
            current = dst
            while predecessors[current] != self.addr and predecessors[current] is not None:
                current = predecessors[current]
            
            # Find the port to the first hop
            for port, (addr, _) in self.neighbors.items():
                if addr == current:
                    self.forwarding_table[dst] = port
                    break

    def __repr__(self):
        """Representation for debugging in the network visualizer."""
        return f"LSrouter(addr={self.addr}, neighbors={list(self.neighbors.values())}, routes={self.forwarding_table})"
