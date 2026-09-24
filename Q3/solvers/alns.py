"""ALNS-RSA driver: destroy/reinsert transport groups, repair via relay MILP."""
from math import exp,inf
from random import Random
from config import RANDOM_SEED
from solvers.transport import pack_by_node,schedule,transport_diagnostics
from solvers.relay import sample_gaps,rank_sites,milp_schedule

def objective(ins,trips):
    delay=sum(b.weight*max(0,t.delivery-b.due) for t in trips for b in t.boxes)
    return delay/1000+max(t.return_time for t in trips)/100+sum(t.energy for t in trips)+len(trips)*5

def search(ins,terrain,iterations=8,relay_time_limit=30):
    rng=Random(RANDOM_SEED)
    current_groups=pack_by_node(ins,0)
    current_score=inf
    best=None
    temperature=50.0
    stagnation=0
    log=[]
    for iteration in range(max(1,iterations)):
        if iteration<3:
            proposal=pack_by_node(ins,iteration)
        else:
            proposal=list(current_groups)
            # Destroy a short block and reinsert it at a different location.
            a=rng.randrange(len(proposal)); width=min(rng.randint(1,3),len(proposal)-a)
            block=proposal[a:a+width]; del proposal[a:a+width]
            b=rng.randrange(len(proposal)+1); proposal[b:b]=block
        try: trips=schedule(ins,terrain,proposal)
        except ValueError as exc:
            log.append({'iteration':iteration,'status':'transport candidate rejected','reason':str(exc)})
            continue
        diagnostics=transport_diagnostics(ins,trips)
        if diagnostics['missing'] or diagnostics['duplicate_count'] or diagnostics['late_hard']:
            log.append({'iteration':iteration,'status':'hard transport infeasible','late_count':len(diagnostics['late_hard'])})
            continue
        score=objective(ins,trips)
        accepted=score<current_score or rng.random()<exp(min(0,(current_score-score)/max(temperature,1e-9)))
        if accepted: current_groups,current_score=proposal,score
        if best is None or score<best['score']:
            gaps=sample_gaps(ins,terrain,trips,step=60)
            sites=rank_sites(ins,terrain,gaps,limit=300)
            relays,uncovered,relay_info=milp_schedule(ins,gaps,sites,limit_sites=45,time_limit=relay_time_limit)
            candidate={'score':score,'trips':trips,'relays':relays,'uncovered':uncovered,
                       'transport':diagnostics,'relay':relay_info,'gaps':len(gaps)}
            if best is None or (not uncovered and best['uncovered']) or (bool(uncovered)==bool(best['uncovered']) and score<best['score']):
                best=candidate; stagnation=0
            else: stagnation+=1
            log.append({'iteration':iteration,'status':relay_info['status'],'score':score,'gap_samples':len(gaps),'uncovered':len(uncovered)})
            if not uncovered: break
        else:
            stagnation+=1
            log.append({'iteration':iteration,'status':'not improved','score':score})
        temperature*=0.90
        if stagnation>=3:
            temperature+=0.5*50.0
            stagnation=0
    return best,log
