"""Load the supplied Q3 workbooks without silently inventing inputs."""
from dataclasses import dataclass
from pathlib import Path
import openpyxl
from config import DATA

@dataclass(frozen=True)
class Node:
    id: str
    lon: float
    lat: float
    ground: float

@dataclass(frozen=True)
class Box:
    id: str
    node: str
    kind: str
    kg: float
    m3: float
    first: bool
    hard: float | None
    due: float
    weight: float

@dataclass(frozen=True)
class TransportType:
    id: str
    empty_kg: float
    max_kg: float
    max_m3: float
    speed: float
    range_empty: float
    range_full: float
    battery_kwh: float
    reserve: float
    prep: float
    load_per_box: float
    handover: float
    handover_per_box: float
    climb: float
    descend: float
    efficiency: float

@dataclass(frozen=True)
class RelayType:
    kg: float
    speed: float
    cruise_kw: float
    battery_kwh: float
    reserve: float
    prep: float
    link: float
    turn: float
    climb: float
    descend: float
    efficiency: float
    hover_kw: float
    comm_kw: float
    max_agl: float

@dataclass(frozen=True)
class Endpoint:
    power_dbm: float
    gain_dbi: float

@dataclass
class Instance:
    nodes: dict[str, Node]
    boxes: list[Box]
    types: dict[str, TransportType]
    drones: dict[str, str]
    batteries: dict[str, list[str]]
    charge_time: dict[str, float]
    relay_type: RelayType
    relays: list[str]
    relay_batteries: list[str]
    relay_charge_time: float
    endpoints: dict[str, Endpoint]
    freq_mhz: float
    system_loss_db: float
    obstacle_db: float
    sensitivity_dbm: float
    margin_db: float
    gateway_agl: float

def rows(filename: str, sheet: str = '数据'):
    wb=openpyxl.load_workbook(DATA / filename, read_only=True, data_only=True)
    try:
        return list(wb[sheet].values)
    finally:
        wb.close()

def load() -> Instance:
    nr = rows('调度中心与服务区.xlsx')
    nodes = {str(r[0]): Node(str(r[0]),float(r[2]),float(r[3]),float(r[4]))
             for r in nr if isinstance(r[0],str) and (r[0]=='O01' or r[0].startswith('S0')) and isinstance(r[2],(int,float))}
    br = rows('物资需求与配送时限.xlsx','逐箱货箱清单')
    boxes=[]
    for r in br[1:]:
        if not r[0]: continue
        hard=float(r[6]) if isinstance(r[6],(int,float)) else None
        if r[2]=='医疗物资': hard=min(hard if hard is not None else float('inf'),float(r[7]))
        boxes.append(Box(str(r[0]),str(r[1]),str(r[2]),float(r[3]),float(r[4]),r[5]=='是',hard,float(r[7]),float(r[8])))
    tr=rows('运输无人机数据.xlsx')
    types={}
    for r in tr[2:5]:
        types[r[0]]=TransportType(str(r[0]),float(r[2]),float(r[3]),float(r[4]),float(r[5]),float(r[6]),float(r[7]),float(r[8]),float(r[9])/100,float(r[10]),float(r[11]),float(r[12]),float(r[13]),float(r[14]),float(r[15]),float(r[16]))
    drones={str(r[0]):str(r[1]) for r in tr if isinstance(r[0],str) and r[0].startswith('U0')}
    batteries={k:[f'{k}-B{j:02}' for j in range(1,next(int(r[1]) for r in tr if r[0]==k and isinstance(r[1],int))+1)] for k in types}
    charge_time={k:float(next(r[2] for r in tr if r[0]==k and isinstance(r[1],int))) for k in types}
    rr=rows('中继无人机数据.xlsx')
    r=rr[2]
    relay_type=RelayType(float(r[4]),float(r[5]),float(r[6]),float(r[7]),float(r[8])/100,float(r[9]),float(r[10]),float(r[11]),float(r[12]),float(r[13]),float(r[14]),float(r[16]),float(r[17]),float(r[18]))
    relays=[str(r[0]) for r in rr if isinstance(r[0],str) and r[0].startswith('R0')]
    count=int(next(r[1] for r in rr if r[0]=='R' and isinstance(r[1],int)))
    relay_batteries=[f'R-B{j:02}' for j in range(1,count+1)]
    relay_charge_time=float(next(r[2] for r in rr if r[0]=='R' and isinstance(r[1],int)))
    cr=rows('通信链路参数.xlsx')
    values={(r[0],r[1]):r[4] for r in cr if isinstance(r[0],str) and isinstance(r[1],str)}
    def val(cat, prefix): return float(next(v for (c,n),v in values.items() if c==cat and n.startswith(prefix)))
    endpoints={name:Endpoint(val(cat,'发射功率'),val(cat,'天线增益')) for name,cat in {'T':'运输无人机','A':'中继接入端','B':'中继回传端','G':'固定网关 G01'}.items()}
    return Instance(nodes,boxes,types,drones,batteries,charge_time,relay_type,relays,relay_batteries,relay_charge_time,endpoints,val('传播参数','载波频率'),val('传播参数','系统损耗'),val('传播参数','地形遮挡'),val('接收参数','接收灵敏度'),val('接收参数','衰落裕量'),val('固定网关 G01','天线离地'))
