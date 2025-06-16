'''
Layered 3D scan implementation.
'''
from typing import Tuple
import numpy as np
from packets import create_3d_layers_packet
from comms import SerialManager
from .base import Scanner

class ThreeDLayerScanner(Scanner):
    '''
    Perform a 2D raster scan across multiple Z-layers thus createing a 3D volume. 
    '''
    def __init__(self,
                 comms: SerialManager,
                 x_neg: int,
                 x_pos: int,
                 y_neg: int,
                 y_pos: int,
                 xy_inc: int,
                 z_neg: int,
                 z_pos: int,
                 z_inc: int,
                 frequency_hz: int) -> None:
        self.comms = comms
        self.params = (
            x_neg,
            x_pos,
            y_neg,
            y_pos,
            xy_inc,
            z_neg,
            z_pos,
            z_inc,
            frequency_hz
        )
    def run(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        pkt = create_3d_layers_packet(*self.params)
        self.comms.write_packet(pkt)

        # Unpack params for dimensions
        x_neg, x_pos, y_neg, y_pos, xy_inc, z_neg, z_pos, z_inc, _ = self.params
        XY_SCALE = 1.2269938650306749
        Z_SCALE = 0.5

        x_count = (x_pos - x_neg) // xy_inc + 1
        y_count = (y_pos - y_neg) // xy_inc + 1
        z_count = (z_pos - z_neg) // z_inc + 1

        x_um = np.linspace(x_neg, x_pos, x_count) * XY_SCALE
        y_um = np.linspace(y_neg, y_pos, y_count) * XY_SCALE
        z_um = np.linspace(z_neg, z_pos, z_count) * Z_SCALE

        volume = np.zeros((x_count, y_count, z_count), dtype=np.uint16)

        # Read number of layers from Arduino
        nz = self.comms.read_count() or 0
        for iz in range(min(nz, z_count)):
            for _ in range(x_count):
                col_idx = self.comms.read_count()
                row_count = self.comms.read_count()
                raw = self.comms.read_block(row_count * 2)
                col_pd = np.frombuffer(raw, dtype='>u2')
                volume[col_idx, :, iz] = col_pd
            print(f'Received layer {iz + 1}/{nz}.')

        return x_um, y_um, z_um, volume