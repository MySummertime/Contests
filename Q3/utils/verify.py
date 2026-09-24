"""Independent safety gate. Never export a plan that fails any check."""
from collections import Counter,defaultdict
from math import ceil
import numpy as np
from core.physics import direct_margin,relay_margin,charge_seconds

def verify(ins,terrain,trips,relays,step=5.0):
    issues=[]; segments=[]
    counts=Counter(b.id for t in trips for b in t.boxes)
    for box in ins.boxes:
        if counts[box.id]!=1: issues.append(f'{box.id}: delivery count {counts[box.id]}')
    for trip in trips:
        if any(b.hard is not None and trip.delivery>b.hard+1e-6 for b in trip.boxes):
            issues.append(f'{trip.id}: hard deadline')
        typ=ins.types[trip.type_id]
        if trip.energy>(1-typ.reserve)*typ.battery_kwh+1e-6: issues.append(f'{trip.id}: reserve SOC')
        if sum(b.kg for b in trip.boxes)>typ.max_kg+1e-6 or sum(b.m3 for b in trip.boxes)>typ.max_m3+1e-6:
            issues.append(f'{trip.id}: load')
    for collection,kind in ((trips,'drone'),(trips,'battery'),(relays,'drone'),(relays,'battery')):
        by=defaultdict(list)
        for task in collection: by[getattr(task,kind)].append(task)
        for resource,tasks in by.items():
            tasks.sort(key=lambda t:t.start)
            for a,b in zip(tasks,tasks[1:]):
                end=a.return_time
                if kind=='battery':
                    cap=ins.types[a.type_id].battery_kwh if a in trips else ins.relay_type.battery_kwh
                    full=ins.charge_time[a.type_id] if a in trips else ins.relay_charge_time
                    end+=charge_seconds(1-a.energy/cap,full)
                elif a in relays: end+=ins.relay_type.turn
                if b.start+1e-5<end: issues.append(f'{resource}: overlapping {a.id},{b.id}')
    for r in relays:
        if r.energy>(1-ins.relay_type.reserve)*ins.relay_type.battery_kwh+1e-5:
            issues.append(f'{r.id}: relay reserve SOC')
        agl=r.site.alt-terrain.ground(r.site.lon,r.site.lat)
        if agl>ins.relay_type.max_agl+1e-5 or agl<0: issues.append(f'{r.id}: hover height')
    # Numerical audit at every phase endpoint and at a fine temporal grid.
    for trip in trips:
        critical=[trip.takeoff,trip.takeoff+trip.outbound.climb_s,
                  trip.takeoff+trip.outbound.climb_s+trip.outbound.cruise_s,
                  trip.service_start,trip.delivery,trip.delivery+trip.inbound.climb_s,
                  trip.delivery+trip.inbound.climb_s+trip.inbound.cruise_s,trip.return_time]
        times=sorted(set(float(x) for x in np.r_[np.arange(trip.takeoff,trip.return_time,step),critical]))
        states=[]
        for t in times:
            point=trip.position(t,ins)
            direct=direct_margin(terrain,point,ins)
            if direct>=0: states.append((t,'直连',None)); continue
            available=[r for r in relays if r.link_done-1e-7<=t<=r.service_end+1e-7 and relay_margin(terrain,point,r.site,ins)>=0]
            if available: states.append((t,'中继',available[0].id))
            else:
                states.append((t,'中断',None))
                if len(issues)<50: issues.append(f'{trip.id}: communication loss at {t:.2f}s')
        for (a,method,rid),(b,next_method,next_rid) in zip(states,states[1:]):
            if b<=a: continue
            phase='投送' if a>=trip.service_start and b<=trip.delivery else '飞行'
            if segments and segments[-1][0]==trip.id and segments[-1][1]==phase and segments[-1][4]==method and segments[-1][5]==rid and abs(segments[-1][3]-a)<1e-5:
                old=segments.pop(); segments.append((trip.id,phase,old[2],b,method,rid))
            else: segments.append((trip.id,phase,a,b,method,rid))
    # Sampling alone does not prove continuous communication. Keep this explicit.
    return {'ok':not issues,'issues':issues,'communication_rows':segments,'audit_step_s':step,
            'continuous_certified':False}
