"""Question 3 paths and reproducible search settings."""
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
DATA = ROOT / 'data' / '无人机应急物资运输基础数据'
DEM = ROOT / 'data' / '镇龙乡地理空间数据' / '镇龙乡及周边地理数据' / '数字高程模型数据（DEM）' / '镇龙乡及周边30米DEM.tif'
RESULTS = HERE / 'results'
TEMPLATE = RESULTS / 'res.xlsx'
RANDOM_SEED = 20260923
TIME_STEP = 10.0  # seconds for diagnostic sampling; interval certification is separate.
MAX_ALNS_ITER = 150
RELAY_GRID_M = 750.0
RELAY_HEIGHTS_M = (100.0, 200.0, 300.0)
BEAM_WIDTH = 16
