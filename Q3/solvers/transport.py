"""Priority-aware trip construction and battery/airframe dispatch."""
from collections import defaultdict
from dataclasses import dataclass
from math import inf
from core.data import Box, Instance
from core.physics import Terrain, Flight, Point, node_point, transport_flight, interpolate_flight, charge_seconds

@dataclass
class Trip:
    id: str
    boxes: tuple[Box,...]
    node: str
    type_id: str
    drone: str
    battery: str
    start: float
    takeoff: float
    delivery: float
    return_time: float
    energy: float
    outbound: Flight
    inbound: Flight
    service_start: float

    def position(self,t,ins:Instance):
        o=node_point(ins.nodes['O01']); dest=node_point(ins.nodes[self.node])
        if t<=self.takeoff: return o
        if t<self.takeoff+self.outbound.duration:
            return interpolate_flight(o,dest,self.outbound,t-self.takeoff)
        if t<=self.delivery: return dest
        if t<self.return_time:
            return interpolate_flight(dest,o,self.inbound,t-self.delivery)
        return o

def priority(box:Box):
    return (box.hard is None,box.hard if box.hard is not None else box.due,-box.weight,box.id)

def pack_by_node(ins:Instance,mode:int=0):
    """Produce physical candidates. Small first-batch loads permit concurrent early launches."""
    groups=[]
    by_node=defaultdict(list)
    for box in ins.boxes: by_node[box.node].append(box)
    for node,all_boxes in by_node.items():
        urgent=sorted((b for b in all_boxes if b.hard is not None),key=priority)
        remaining=sorted((b for b in all_boxes if b.hard is None),key=priority)
        if urgent:
            # Mode changes first-wave packing, giving ALNS a meaningful neighborhood.
            wave=[]
            cap=(25 if mode%3==0 else 30 if mode%3==1 else 80)
            volume=(0.06 if mode%3==0 else 0.073 if mode%3==1 else 0.25)
            for b in urgent[:]:
                if sum(x.kg for x in wave)+b.kg<=cap and sum(x.m3 for x in wave)+b.m3<=volume:
                    wave.append(b)
            if wave:
                groups.append(tuple(wave)); urgent=[b for b in urgent if b not in wave]
        todo=urgent+remaining
        while todo:
            group=[]
            for b in list(todo):
                if sum(x.kg for x in group)+b.kg<=80 and sum(x.m3 for x in group)+b.m3<=0.25:
                    group.append(b); todo.remove(b)
            if not group: raise ValueError(f'Unpackable box at {node}')
            groups.append(tuple(group))
    groups.sort(key=lambda g:(min((b.hard for b in g if b.hard is not None),default=inf),-sum(b.weight for b in g),g[0].node))
    return groups

def schedule(ins:Instance,terrain:Terrain,groups):
    """Dispatch each trip to the earliest feasible (airframe,battery) pair."""
    drone_ready={d:0.0 for d in ins.drones}
    battery_ready={b:0.0 for bs in ins.batteries.values() for b in bs}
    trips=[]
    o=node_point(ins.nodes['O01'])
    for group in groups:
        node=group[0].node
        if any(b.node!=node for b in group): raise ValueError('Only single-area groups supported')
        dest=node_point(ins.nodes[node])
        weight=sum(b.kg for b in group); volume=sum(b.m3 for b in group)
        choices=[]
        for drone,typ in ins.drones.items():
            m=ins.types[typ]
            if weight>m.max_kg+1e-8 or volume>m.max_m3+1e-8: continue
            out=transport_flight(terrain,o,dest,m,weight)
            back=transport_flight(terrain,dest,o,m,0)
            energy=out.energy+back.energy
            if energy>(1-m.reserve)*m.battery_kwh+1e-8: continue
            prep=m.prep+len(group)*m.load_per_box
            service=m.handover+len(group)*m.handover_per_box
            for battery in ins.batteries[typ]:
                start=max(drone_ready[drone],battery_ready[battery])
                takeoff=start+prep
                delivery=takeoff+out.duration+service
                finish=delivery+back.duration
                violations=sum(max(0,delivery-b.hard) for b in group if b.hard is not None)
                tardy=sum(b.weight*max(0,delivery-b.due) for b in group)
                score=(violations>1e-6,violations,tardy,finish,energy)
                choices.append((score,drone,battery,typ,start,takeoff,delivery,finish,energy,out,back,service))
        if not choices: raise ValueError(f'No physical transport for {node}: {[b.id for b in group]}')
        _,drone,battery,typ,start,takeoff,delivery,finish,energy,out,back,service=min(choices,key=lambda x:x[0])
        trips.append(Trip(f'T{len(trips)+1:03}',group,node,typ,drone,battery,start,takeoff,delivery,finish,energy,out,back,delivery-service))
        drone_ready[drone]=finish
        battery_ready[battery]=finish+charge_seconds(1-energy/ins.types[typ].battery_kwh,ins.charge_time[typ])
    return trips

def transport_diagnostics(ins:Instance,trips):
    delivered=[b.id for t in trips for b in t.boxes]
    expected={b.id for b in ins.boxes}
    missing=expected-set(delivered)
    duplicate=len(delivered)-len(set(delivered))
    late=[(b.id,t.delivery,b.hard) for t in trips for b in t.boxes if b.hard is not None and t.delivery>b.hard+1e-5]
    return {'trip_count':len(trips),'missing':sorted(missing),'duplicate_count':duplicate,'late_hard':late,
            'max_return_s':max((t.return_time for t in trips),default=0),
            'energy_kwh':sum(t.energy for t in trips)}
