####################################################
# DVrouter.py
# Name:
# HUID:
#####################################################

import json
from router import Router
from packet import Packet


class DVrouter(Router):
    """Distance vector routing protocol implementation.

    Add your own class fields and initialization code (e.g. to create forwarding table
    data structures). See the Router base class for docstrings of the methods to
    override.
    """

    def __init__(self, addr, heartbeat_time):
        Router.__init__(self, addr)  # Initialize base class - DO NOT REMOVE
        self.heartbeat_time = heartbeat_time
        self.last_time = 0
        
        # Distance vector: dv[dest] = cost to reach dest via our own routes
        self.dv = {addr: 0}
        
        # Neighbor costs: neighbors[endpoint] = (port, cost)
        self.neighbors = {}
        
        # Distance vectors from neighbors: received_dvs[endpoint] = neighbor's dv
        self.received_dvs = {}
        
        # Forwarding table: forwarding_table[dst] = (port, next_hop)
        self.forwarding_table = {}
        
        # Infinity value for unreachable destinations
        self.INFINITY = 16

    def _update_dv(self):
        """Update distance vector using Bellman-Ford algorithm."""
        old_dv = dict(self.dv)
        
        # Update distance to each destination
        for dest in set(self.dv.keys()) | set().union(*[set(dvs.keys()) for dvs in self.received_dvs.values()]):
            if dest == self.addr:
                self.dv[dest] = 0
            else:
                # Find minimum cost path through any neighbor
                min_cost = self.INFINITY
                best_neighbor = None
                
                for neighbor, (port, link_cost) in self.neighbors.items():
                    if neighbor in self.received_dvs:
                        neighbor_dv = self.received_dvs[neighbor]
                        if dest in neighbor_dv:
                            # Cost = link cost + neighbor's distance to dest
                            cost = link_cost + neighbor_dv[dest]
                            if cost < min_cost:
                                min_cost = cost
                                best_neighbor = neighbor
                
                # Update our DV and forwarding table
                self.dv[dest] = min_cost
                if min_cost < self.INFINITY:
                    port, _ = self.neighbors[best_neighbor]
                    self.forwarding_table[dest] = (port, best_neighbor)
                elif dest in self.forwarding_table:
                    del self.forwarding_table[dest]
        
        # Return True if DV changed
        return self.dv != old_dv

    def _broadcast_dv(self):
        """Broadcast distance vector to all neighbors using split horizon."""
        if not self.neighbors:
            return
        
        for neighbor, (port, _) in self.neighbors.items():
            # Create distance vector with split horizon
            # Don't advertise routes that go back through this neighbor
            advertised_dv = {}
            for dest, cost in self.dv.items():
                if dest == self.addr:
                    advertised_dv[dest] = 0
                elif dest in self.forwarding_table:
                    next_port, next_hop = self.forwarding_table[dest]
                    # Split horizon: don't send if this neighbor is the next hop
                    if next_hop != neighbor:
                        advertised_dv[dest] = cost
                else:
                    # Unreachable destination - could send INFINITY or skip
                    advertised_dv[dest] = self.INFINITY
            
            # Send routing packet with DV
            content = json.dumps(advertised_dv)
            packet = Packet(Packet.ROUTING, self.addr, self.addr, content)
            self.send(port, packet)

    def handle_packet(self, port, packet):
        """Process incoming packet."""
        if packet.is_traceroute:
            # Forward traceroute packet based on forwarding table
            if packet.dst_addr in self.forwarding_table:
                out_port, _ = self.forwarding_table[packet.dst_addr]
                self.send(out_port, packet)
        else:
            # This is a routing packet containing a distance vector
            try:
                received_dv = json.loads(packet.content)
                
                # Find which neighbor sent this
                neighbor = None
                for n, (p, _) in self.neighbors.items():
                    if p == port:
                        neighbor = n
                        break
                
                if neighbor and neighbor in self.neighbors:
                    # Update received DV from this neighbor
                    old_received_dv = self.received_dvs.get(neighbor, {})
                    self.received_dvs[neighbor] = received_dv
                    
                    # If DV changed, update our own DV and broadcast
                    if received_dv != old_received_dv:
                        if self._update_dv():
                            self._broadcast_dv()
            except (json.JSONDecodeError, KeyError):
                # Invalid packet, ignore
                pass

    def handle_new_link(self, port, endpoint, cost):
        """Handle new link."""
        self.neighbors[endpoint] = (port, cost)
        
        # Initialize distance vector for new neighbor
        if endpoint not in self.received_dvs:
            self.received_dvs[endpoint] = {endpoint: 0}
        
        # Update our distance vector
        self.dv[endpoint] = cost
        self.forwarding_table[endpoint] = (port, endpoint)
        
        # Broadcast updated DV
        self._broadcast_dv()

    def handle_remove_link(self, port):
        """Handle removed link."""
        # Find which neighbor this port connected to
        removed_neighbor = None
        for neighbor, (p, _) in list(self.neighbors.items()):
            if p == port:
                removed_neighbor = neighbor
                break
        
        if removed_neighbor:
            del self.neighbors[removed_neighbor]
            del self.received_dvs[removed_neighbor]
            
            # Re-run Bellman-Ford to update DV
            if self._update_dv():
                self._broadcast_dv()

    def handle_time(self, time_ms):
        """Handle current time."""
        if time_ms - self.last_time >= self.heartbeat_time:
            self.last_time = time_ms
            # Periodic broadcast of distance vector
            self._broadcast_dv()

    def __repr__(self):
        """Representation for debugging in the network visualizer."""
        return f"DVrouter(addr={self.addr}, dv={self.dv})"