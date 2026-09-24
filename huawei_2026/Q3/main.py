"""Run Q3 in the local conda environment; export only validated results."""
import argparse
import sys
from pathlib import Path
from core.data import load
from core.physics import Terrain
from solvers.alns import search
from utils.verify import verify
from utils.output import save_workbook,save_diagnostic_figure,save_status

def main(argv=None):
    parser=argparse.ArgumentParser(description='Q3 joint transport/relay solver')
    parser.add_argument('--iterations',type=int,default=8)
    parser.add_argument('--milp-time-limit',type=int,default=30)
    parser.add_argument('--audit-step',type=float,default=5.0)
    args=parser.parse_args(argv)
    if Path(sys.prefix).name.lower() != 'huawei2026' or not (Path(sys.prefix) / 'conda-meta').is_dir():
        raise RuntimeError('Activate the shared conda environment named huawei2026 before running this program')
    try:
        import gurobipy as gp
        probe=gp.Model('q3_license_probe')
        probe.Params.OutputFlag=0
        probe.dispose()
    except Exception as exc:
        print(f'Gurobi unavailable: {exc}. No result files were changed.',file=sys.stderr)
        return 3
    ins=load(); terrain=Terrain()
    best,log=search(ins,terrain,args.iterations,args.milp_time_limit)
    if best is None:
        payload={'status':'NO_TRANSPORT_CANDIDATE','search':log}
        save_status(payload)
        print(payload['status'])
        return 2
    audit=verify(ins,terrain,best['trips'],best['relays'],args.audit_step)
    # The current numerical radio audit does not establish interval-wide LOS.
    payload={'status':'FEASIBLE' if audit['ok'] and audit['continuous_certified'] else 'UNCERTIFIED',
             'transport':best['transport'],'relay':best['relay'],
             'gap_samples':best['gaps'],'milp_uncovered_samples':len(best['uncovered']),
             'audit_issues':audit['issues'],'audit_step_s':audit['audit_step_s'],
             'continuous_certified':audit['continuous_certified'],'search':log}
    save_status(payload)
    figure=save_diagnostic_figure(ins,best['trips'],best['relays'],audit)
    if payload['status']=='FEASIBLE':
        output=save_workbook(best['relays'],audit)
        print(f'VALIDATED: {output}; {figure}')
        return 0
    print(f'UNCERTIFIED: template unchanged; diagnostics: {figure}')
    return 2

if __name__=='__main__': raise SystemExit(main())
