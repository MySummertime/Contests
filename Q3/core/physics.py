"""DEM, prescribed three-phase flights, energy, and bidirectional radio link."""
from dataclasses import dataclass
from functools import lru_cache
from math import ceil, hypot, log10
import numpy as np
import rasterio
from pyproj import Geod
from config import DEM
from core.data import Node, Instance, TransportType

G=9.80665
GEOD=Geod(ellps='WGS84')

@dataclass(frozen=True)
class Point:
    lon: float
    lat: float
    alt: float

@dataclass(frozen=True)
class Flight:
    duration: float
    energy: float
    cruise_alt: float
    climb_s: float
    cruise_s: float
    descend_s: float
    distance_m: float

class Terrain:
    def __init__(self, filename=DEM):
        with rasterio.open(filename) as src:
            self.array=src.read(1)
            self.transform=src.transform
            self.bounds=src.bounds
            self.nodata=src.nodata
        self.geod=GEOD

    def ground(self,lon,lat):
        col,row=(~self.transform)*(lon,lat)
        col,row=int(col),int(row)
        if row<0 or col<0 or row>=self.array.shape[0] or col>=self.array.shape[1]:
            raise ValueError(f'Point outside DEM: {lon},{lat}')
        val=float(self.array[row,col])
        if not np.isfinite(val) or (self.nodata is not None and val==self.nodata):
            raise ValueError(f'No DEM elevation at {lon},{lat}')
        return val

    @lru_cache(maxsize=20000)
    def profile(self,lon1,lat1,lon2,lat2):
        _,_,dist=self.geod.inv(lon1,lat1,lon2,lat2)
        n=max(1,ceil(dist/15))
        lons=np.linspace(lon1,lon2,n+1)
        lats=np.linspace(lat1,lat2,n+1)
        heights=np.array([self.ground(float(x),float(y)) for x,y in zip(lons,lats)])
        return dist,lons,lats,heights

    def distance(self,a:Point,b:Point):
        _,_,d=self.geod.inv(a.lon,a.lat,b.lon,b.lat)
        return d

    def cruise_alt(self,a:Point,b:Point):
        return max(self.profile(a.lon,a.lat,b.lon,b.lat)[3])+50.0

    def los(self,a:Point,b:Point):
        _,_,_,h=self.profile(a.lon,a.lat,b.lon,b.lat)
        straight=np.linspace(a.alt,b.alt,len(h))
        return bool(np.all(straight[1:-1]>h[1:-1]))

def node_point(node:Node):
    return Point(node.lon,node.lat,node.ground+(0 if node.id=='O01' else 30))

def transport_flight(terrain:Terrain,a:Point,b:Point,m:TransportType,payload_kg:float):
    if payload_kg< -1e-9 or payload_kg>m.max_kg+1e-9: raise ValueError('Invalid payload')
    dist=terrain.distance(a,b)
    alt=terrain.cruise_alt(a,b)
    up=max(0,alt-a.alt)
    down=max(0,alt-b.alt)
    ts=(up/m.climb,dist/m.speed,down/m.descend)
    equivalent_range=m.range_empty-(m.range_empty-m.range_full)*(payload_kg/m.max_kg)**1.5
    energy=m.battery_kwh*dist/equivalent_range+(m.empty_kg+payload_kg)*G*up/(3.6e6*m.efficiency)
    return Flight(sum(ts),energy,alt,*ts,dist)

def relay_flight(terrain:Terrain,a:Point,b:Point,ins:Instance):
    r=ins.relay_type
    dist=terrain.distance(a,b)
    alt=terrain.cruise_alt(a,b)
    up=max(0,alt-a.alt)
    down=max(0,alt-b.alt)
    ts=(up/r.climb,dist/r.speed,down/r.descend)
    energy=r.cruise_kw*ts[1]/3600+r.kg*G*up/(3.6e6*r.efficiency)
    return Flight(sum(ts),energy,alt,*ts,dist)

def interpolate_flight(a:Point,b:Point,flight:Flight,seconds:float):
    t=min(max(0.0,seconds),flight.duration)
    if t<=flight.climb_s:
        z=a.alt+(flight.cruise_alt-a.alt)*t/max(flight.climb_s,1e-9)
        return Point(a.lon,a.lat,z)
    if t<=flight.climb_s+flight.cruise_s:
        f=(t-flight.climb_s)/max(flight.cruise_s,1e-9)
        return Point(a.lon+(b.lon-a.lon)*f,a.lat+(b.lat-a.lat)*f,flight.cruise_alt)
    f=(t-flight.climb_s-flight.cruise_s)/max(flight.descend_s,1e-9)
    return Point(b.lon,b.lat,flight.cruise_alt+(b.alt-flight.cruise_alt)*f)

def link_margin_db(terrain:Terrain,a:Point,b:Point,ea,eb,ins:Instance):
    horizontal=terrain.distance(a,b)
    distance_km=hypot(horizontal,a.alt-b.alt)/1000
    loss=32.45+20*log10(ins.freq_mhz)+20*log10(max(distance_km,1e-9))
    if not terrain.los(a,b): loss+=ins.obstacle_db
    allowed_ab=ea.power_dbm+ea.gain_dbi+eb.gain_dbi-ins.system_loss_db-ins.sensitivity_dbm-ins.margin_db
    allowed_ba=eb.power_dbm+eb.gain_dbi+ea.gain_dbi-ins.system_loss_db-ins.sensitivity_dbm-ins.margin_db
    return min(allowed_ab,allowed_ba)-loss

def direct_margin(terrain,where,ins):
    o=ins.nodes['O01']; gateway=Point(o.lon,o.lat,o.ground+ins.gateway_agl)
    return link_margin_db(terrain,where,gateway,ins.endpoints['T'],ins.endpoints['G'],ins)

def relay_margin(terrain,where,relay,ins):
    o=ins.nodes['O01']; gateway=Point(o.lon,o.lat,o.ground+ins.gateway_agl)
    return min(link_margin_db(terrain,where,relay,ins.endpoints['T'],ins.endpoints['A'],ins),
               link_margin_db(terrain,relay,gateway,ins.endpoints['B'],ins.endpoints['G'],ins))

def charge_seconds(soc_end,full_time):
    s=max(0,min(1,soc_end))
    return full_time*(0.65*max(0,0.9-s)/0.9+0.35*max(0,1-max(s,0.9))/0.1)
