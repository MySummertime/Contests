"""Relay siting and rotation for transport direct-link gaps."""
from dataclasses import dataclass
from math import inf
import numpy as np
from core.data import Instance
from core.physics import Terrain,Point,node_point,relay_flight,relay_margin,direct_margin,charge_seconds

@dataclass(frozen=True)
class Gap:
    trip: str
    time: float
    position: Point

@dataclass
class RelayTrip:
    id: str
    drone: str
    battery: str
    site: Point
    start: float
    link_done: float
    service_end: float
    return_time: float
    energy: float

def sample_gaps(ins:Instance,terrain:Terrain,trips,step=30):
    gaps=[]
    for trip in trips:
        times=list(np.arange(trip.takeoff,trip.return_time,step))+[trip.return_time]
        # Include phase boundaries exactly; diagnostic samples alone are not a proof.
        times += [trip.takeoff+trip.outbound.climb_s,
                  trip.takeoff+trip.outbound.climb_s+trip.outbound.cruise_s,
                  trip.service_start,trip.delivery,
                  trip.delivery+trip.inbound.climb_s,
                  trip.delivery+trip.inbound.climb_s+trip.inbound.cruise_s]
        for t in sorted(set(float(x) for x in times if trip.takeoff<=x<=trip.return_time)):
            point=trip.position(t,ins)
            if direct_margin(terrain,point,ins)<0: gaps.append(Gap(trip.id,t,point))
    return gaps

def sites(ins:Instance,terrain:Terrain):
    o=ins.nodes['O01']; result=[]; seen=set()
    for node in ins.nodes.values():
        for f in (0.4,0.65,1.0):
            lon=o.lon+(node.lon-o.lon)*f; lat=o.lat+(node.lat-o.lat)*f
            key=(round(lon,5),round(lat,5))
            if key in seen: continue
            seen.add(key)
            try: ground=terrain.ground(lon,lat)
            except ValueError: continue
            for agl in (150,225,300):
                result.append(Point(lon,lat,ground+agl))
    return result

def site_properties(ins,terrain,site):
    o=node_point(ins.nodes['O01'])
    outbound=relay_flight(terrain,o,site,ins)
    back=relay_flight(terrain,site,o,ins)
    travel_energy=outbound.energy+back.energy
    usable=(1-ins.relay_type.reserve)*ins.relay_type.battery_kwh-travel_energy
    endurance=max(0,usable*3600/(ins.relay_type.hover_kw+ins.relay_type.comm_kw))
    return outbound,back,travel_energy,endurance

def rank_sites(ins:Instance,terrain:Terrain,gaps:list[Gap],limit=30):
    """Beam-style ranking by physically serviceable gap points and energy."""
    ranked=[]
    for site in sites(ins,terrain):
        out,back,travel,endurance=site_properties(ins,terrain,site)
        if endurance<300: continue
        covered=frozenset(i for i,g in enumerate(gaps) if relay_margin(terrain,g.position,site,ins)>=0)
        if not covered: continue
        score=len(covered)/(1+0.15*travel)
        ranked.append((score,site,covered,out,back,travel,endurance))
    ranked.sort(key=lambda row:row[0],reverse=True)
    return ranked[:limit]

def cover_stats(candidates,gaps):
    all_ids=set(range(len(gaps)))
    one=max((len(x[2]) for x in candidates),default=0)
    two=max((len(a[2]|b[2]) for a in candidates for b in candidates),default=0)
    missing=all_ids-set().union(*(x[2] for x in candidates)) if candidates else all_ids
    return {'gaps':len(gaps),'best_one':one,'best_two':two,'not_coverable_any_site':len(missing)}

def construct_schedule(ins:Instance,terrain:Terrain,gaps:list[Gap],candidates):
    """Greedy time-window set cover with exact airframe and recharging clocks."""
    uncovered=set(range(len(gaps)))
    ready_drone={d:0.0 for d in ins.relays}
    ready_battery={b:0.0 for b in ins.relay_batteries}
    sorties=[]
    if not gaps: return sorties,uncovered
    sorted_candidates=sorted(candidates,key=lambda row:row[0],reverse=True)
    for _ in range(30):
        if not uncovered: break
        first=min(uncovered,key=lambda i:gaps[i].time)
        options=[]
        for _,site,covered,out,back,travel,endurance in sorted_candidates:
            if first not in covered: continue
            for drone,drone_time in ready_drone.items():
                for battery,battery_time in ready_battery.items():
                    earliest=max(drone_time,battery_time)
                    link_done=earliest+ins.relay_type.prep+out.duration+ins.relay_type.link
                    if link_done>gaps[first].time+1e-7: continue
                    latest=link_done+endurance
                    window={i for i in uncovered & covered if link_done<=gaps[i].time<=latest}
                    if not window: continue
                    last=max(gaps[i].time for i in window)
                    # Return just after the last covered gap, reserving a safety margin.
                    service_end=min(latest,last+20)
                    energy=travel+(ins.relay_type.hover_kw+ins.relay_type.comm_kw)*(service_end-link_done)/3600
                    score=(len(window),-energy,-earliest)
                    options.append((score,site,drone,battery,earliest,link_done,service_end,back,energy,window))
        if not options: break
        _,site,drone,battery,start,link_done,end,back,energy,window=max(options,key=lambda x:x[0])
        finish=end+back.duration
        sorties.append(RelayTrip(f'R{len(sorties)+1:03}',drone,battery,site,start,link_done,end,finish,energy))
        ready_drone[drone]=finish+ins.relay_type.turn
        ready_battery[battery]=finish+charge_seconds(1-energy/ins.relay_type.battery_kwh,ins.relay_charge_time)
        uncovered-=window
    return sorties,uncovered

def milp_schedule(ins:Instance,gaps:list[Gap],ranked,limit_sites=36,time_limit=120):
    """Set-cover MILP over relay site/time windows with two airframes and six batteries."""
    if not gaps: return [],set(),{'status':'no relay required'}
    chosen=list(ranked[:min(12,len(ranked))])
    covered=set().union(*(r[2] for r in chosen))
    remaining=set(range(len(gaps)))-covered
    for r in ranked[12:]:
        if not remaining: break
        if r[2]&remaining:
            chosen.append(r)
            remaining-=r[2]
        if len(chosen)>=limit_sites: break
    if remaining:
        return [],remaining,{'status':'site pool incomplete','sites':len(chosen)}
    horizon=max(g.time for g in gaps)+300
    windows=[]
    for _,site,coverage,out,back,travel,endurance in chosen:
        if endurance<300: continue
        launches=np.arange(0,horizon,450.0)
        for launch in launches:
            service_start=float(launch+ins.relay_type.prep+out.duration+ins.relay_type.link)
            if service_start>horizon: break
            for duration in (900,1800,3000,4500,6000):
                duration=min(float(duration),endurance,horizon-service_start+60)
                if duration<240: continue
                service_end=service_start+duration
                cover=frozenset(i for i in coverage if service_start<=gaps[i].time<=service_end)
                if not cover: continue
                energy=travel+(ins.relay_type.hover_kw+ins.relay_type.comm_kw)*duration/3600
                finish=service_end+back.duration
                recharge=charge_seconds(1-energy/ins.relay_type.battery_kwh,ins.relay_charge_time)
                windows.append((site,float(launch),service_start,service_end,finish,energy,recharge,cover))
    if not windows: return [],set(range(len(gaps))),{'status':'no feasible windows'}
    import gurobipy as gp
    from gurobipy import GRB
    n=len(windows)
    slot=60.0
    slots=int(np.ceil((max(w[4]+w[6] for w in windows)+1)/slot))
    gap_to_windows=[[] for _ in gaps]
    drone_slots=[[] for _ in range(slots)]
    battery_slots=[[] for _ in range(slots)]
    for j,w in enumerate(windows):
        for i in w[7]: gap_to_windows[i].append(j)
        for k in range(int(w[1]//slot),min(slots,int(np.ceil((w[4]+ins.relay_type.turn)/slot)))):
            drone_slots[k].append(j)
        for k in range(int(w[1]//slot),min(slots,int(np.ceil((w[4]+w[6])/slot)))):
            battery_slots[k].append(j)
    no_window=[i for i,js in enumerate(gap_to_windows) if not js]
    if no_window:
        return [],set(no_window),{'status':'gap has no feasible timed window','count':len(no_window),'examples':[(gaps[i].trip,gaps[i].time) for i in no_window[:8]],'windows':n,'sites':len(chosen)}
    model=gp.Model('Q3_relay_coverage')
    model.Params.OutputFlag=0
    model.Params.TimeLimit=time_limit
    model.Params.MIPGap=0.05
    x=model.addVars(n,vtype=GRB.BINARY,name='mission')
    model.setObjective(gp.quicksum((1+0.12*w[5]+0.0001*w[4])*x[j] for j,w in enumerate(windows)),GRB.MINIMIZE)
    for i,js in enumerate(gap_to_windows): model.addConstr(gp.quicksum(x[j] for j in js)>=1,name=f'cover_{i}')
    for k,js in enumerate(drone_slots):
        if js: model.addConstr(gp.quicksum(x[j] for j in js)<=len(ins.relays),name=f'drone_{k}')
    for k,js in enumerate(battery_slots):
        if js: model.addConstr(gp.quicksum(x[j] for j in js)<=len(ins.relay_batteries),name=f'battery_{k}')
    model.optimize()
    if model.SolCount==0:
        return [],set(range(len(gaps))),{'status':f'Gurobi status {model.Status}','windows':n,'sites':len(chosen)}
    selected=[windows[j] for j in range(n) if x[j].X>0.5]
    selected.sort(key=lambda w:w[1])
    ready_drone={d:0.0 for d in ins.relays}
    ready_battery={b:0.0 for b in ins.relay_batteries}
    sorties=[]
    for w in selected:
        drone=next((d for d,r in ready_drone.items() if r<=w[1]+1e-6),None)
        battery=next((b for b,r in ready_battery.items() if r<=w[1]+1e-6),None)
        if drone is None or battery is None:
            return [],set(range(len(gaps))),{'status':'resource coloring failed','windows':n}
        ready_drone[drone]=w[4]+ins.relay_type.turn
        ready_battery[battery]=w[4]+w[6]
        sorties.append(RelayTrip(f'R{len(sorties)+1:03}',drone,battery,w[0],w[1],w[2],w[3],w[4],w[5]))
    uncovered={i for i,g in enumerate(gaps) if not any(i in w[7] for w in selected)}
    return sorties,uncovered,{'status':f'Gurobi status {model.Status}','windows':n,'sites':len(chosen),'mip_gap':model.MIPGap}
