'''
Galvo control and raster scanning implementations.
'''
from typing import Tuple, Optional
import numpy as np
from packets import data_to_bytes, create_galvo_scan_packet, create_show_area_packet
from comms import SerialManager
from .base import Scanner

class GalvoPointScanner(Scanner):
    '''
    Move galvo to a single (X,Y) then read PD once.
    '''
    def __init__(self,
                 comms: SerialManager,
                 x: int,
                 y: int,
                 aux_flag: bool=False) -> None:
        self.comms = comms
        self.x = x
        self.y = y
        self.aux = aux_flag
      
    def run(self) -> Optional[int]:
        pkt = data_to_bytes(self.x, self.y, self.aux)
        self.comms.write_packet(pkt)
        return self.comms.read_pd()
    
class GalvoRasterScanner(Scanner):
    '''
    Perform a full 2D galvo raster scan, returning X, Y axes and PD image.
    '''
    def __init__(self,
                    comms: SerialManager,
                    x_neg: int,
                    x_pos: int,
                    y_neg: int,
                    y_pos: int,
                    increment: int) -> None:
        self.comms = comms
        self.x_neg = x_neg
        self.x_pos = x_pos
        self.y_neg = y_neg
        self.y_pos = y_pos
        self.inc = increment

    def run(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        pkt = create_galvo_scan_packet(self.x_neg, self.x_pos, self.y_neg, self.y_pos, self.inc)
        self.comms.write_packet(pkt)

        # Compute dimensions
        XY_SCALE = 1.2269938650306749
        x_count = (self.x_pos - self.x_neg) // self.inc + 1
        y_count = (self.y_pos - self.y_neg) // self.inc + 1

        x_um = np.linspace(self.x_neg, self.x_pos, x_count) * XY_SCALE
        y_um = np.linspace(self.y_neg, self.y_pos, y_count) * XY_SCALE
        volume = np.zeros((x_count, y_count), dtype=np.uint16)

        for _ in range(x_count):
            col_idx = self.comms.read_count()
            row_count = self.comms.read_count()
            raw = self.comms.read_block(row_count * 2)
            col_pd = np.frombuffer(raw, dtype='>u2')
            volume[col_idx, :] = col_pd

        return x_um, y_um, volume
        
class ShowAreaScanner(Scanner):
    '''
    Outline a rectangular area with galvo loops; waits for 0xAA55 flag.
    '''
    def __init__(self,
                 comms: SerialManager,
                 x_neg: int,
                 x_pos: int,
                 y_neg: int,
                 y_pos: int,
                 increment: int=1,
                 loops: int=10) -> None:
        self.comms = comms
        self.params = (x_neg, x_pos, y_neg, y_pos, increment, loops)

    def run(self) -> bool:
            pkt = create_show_area_packet(*self.params)
            self.comms.write_packet(pkt)
            resp = self.comms.read_block(2)
            return resp == b'\xAA\x55'