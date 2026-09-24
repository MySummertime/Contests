# Q3 transport and relay solver

From the repository root, create and activate the shared conda environment for Q3 and Q4 (Windows or macOS):

```text
conda env create -f environment.yml
conda activate huawei2026
python Q3/main.py --iterations 8 --milp-time-limit 30
```

Input paths are resolved from `Q3/config.py` relative to the checked-out project: `data/` is a sibling of `Q3/`. If Times New Roman is installed in a nonstandard location, set `Q3_TIMES_FONT` to the font file before generating figures.

`core` reads the supplied workbooks and DEM and implements flight, battery, and radio physics. `solvers` constructs transport trips, searches relay sites, and solves a sampled coverage MILP. `utils` independently audits the resulting schedule and owns workbook/figure output. The supplied `results/res.xlsx` is **never overwritten** unless all checks, including continuous radio coverage, are certified.

Current status: the relay MILP uses sampled coverage points. The independent radio audit also samples time. Neither is a proof of continuous terrain-occluded communication, so the present implementation intentionally leaves `continuous_certified=False` and reports `UNCERTIFIED` rather than fabricating a competition result. The workbook therefore remains blank until an interval-wide link verifier is implemented and a feasible schedule is found. A diagnostic PNG and `q3_status.json` are written only after the Gurobi preflight passes; PNGs are 600 dpi, transparent, and use Times New Roman.

Gurobi is required for the relay MILP and must be licensed on the machine that runs the solver. If license initialization fails, the entry point exits before changing results. A valid local license is therefore needed independently on Windows and on the MacBook. The supplied `results/res.xlsx` is a tracked template; generated PNGs, status JSON, local environments, caches and logs are ignored by the repository-level `.gitignore`.
