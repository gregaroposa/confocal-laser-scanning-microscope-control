'''
Z-scan and autofocus scanner implementations.
'''
from typing import Tuple, Optional
import numpy as np
from packets import create_zscan_packet, create_autofocus_packet, create_stepper_packet
from comms import SerialManager
from processing import remove_spikes, savgol_smooth
from .base import Scanner

class ZScanScanner(Scanner):
    '''
    Perform a single Z-scan: sweep the stepper between neg and pos limits and read PD.
    '''
    def __init__(self,
                 comms: SerialManager,
                 neg_limit: int,
                 pos_limit: int,
                 step_increment: int,
                 frequency_hz: int) -> None:
        '''
        Args:
            comms: SerialManager instance for communication with Arduino.
            neg_limit: Negative raw step limit (<=0).
            pos_limit: Positive raw step limit (>=0).
            step_increment: Raw step increment (>=1).
            frequency_hz: Sweep frequency (Hz, multiple of 24).
        '''
        self.comms = comms
        self.neg = neg_limit
        self.pos = pos_limit
        self.inc= step_increment
        self.freq = frequency_hz

    def run(self) -> Tuple[np.ndarray, np.ndarray]:
        '''
        Executes the Z-scan and returns (z_positions_microns, smoothed_pd).
        '''
        pkt = create_zscan_packet(self.neg, self.pos, self.inc, self.freq)
        self.comms.write_packet(pkt)

        count = self.comms.read_count()
        if count is None or count == 0:
            raise RuntimeError("No data received from Arduino during Z-scan.")
        
        raw = self.comms.read_block(count * 4) # 4 bytes per sample (2 for z, 2 for PD)
        zs = np.zeros(count, dtype=np.int16)
        pds = np.zeros(count, dtype=np.int16)
        for i in range(count):
            z_i, pd_i = np.frombuffer(raw[4*i:4*(i+1)], dtype='>i2,>u2')[0]
            zs[i], pds[i] = z_i, pd_i
        
        # Disregard the first measurement
        if count > 0:
            zs = zs[1:]
            pds = pds[1:]

        # Convert to physical units
        Z_SCALE = 0.5 # microns per step
        zs_um = zs * Z_SCALE

        # Clean and smooth the PD signal
        pds_clean = remove_spikes(pds)
        pds_smooth = savgol_smooth(pds_clean)

        return zs_um, pds_smooth
    
class AutofocusScanner(Scanner):
    '''
    Perform an autofocus sweep and compute the best focus position.
    '''
    def __init__(self,
                 comms: SerialManager,
                 neg_limit: int,
                 pos_limit: int,
                 step_increment: int,
                 frequency_hz: int) -> None:
        '''
        Args:
            comms: SerialManager instance for communication with Arduino.
            neg_limit: Negative raw step limit (<=0).
            pos_limit: Positive raw step limit (>=0).
            step_increment: Raw step increment (>=1).
            frequency_hz: Sweep frequency (Hz, multiple of 24).
        '''
        self.comms = comms
        self.neg = neg_limit
        self.pos = pos_limit
        self.inc = step_increment
        self.freq = frequency_hz

    def run(self) -> Tuple[np.ndarray, np.ndarray, Optional[float]]:
        '''
        Executes autofocus: returns (z_um, pds_smooth, best_z_um).
        '''
        pkt = create_autofocus_packet(self.neg, self.pos, self.inc, self.freq)
        self.comms.write_packet(pkt)

        count = self.comms.read_count()
        if count is None or count == 0:
            raise RuntimeError("No data received from Arduino during autofocus.")
        
        raw = self.comms.read_block(count * 4)
        zs = np.zeros(count, dtype=np.int16)
        pds = np.zeros(count, dtype=np.int16)
        for i in range(count):
            z_i = int.from_bytes(raw[4*i:4*i+2], 'big', signed=True)
            pd_i = int.from_bytes(raw[4*i+2:4*i+4], 'big')
            zs[i], pds[i] = z_i, pd_i
        
        # Disregard the first measurement
        if count > 0:
            zs = zs[1:]
            pds = pds[1:]
            
        Z_SCALE = 0.5 # microns per step
        zs_um = zs * Z_SCALE

        pds_clean = remove_spikes(pds)
        pds_smooth = savgol_smooth(pds_clean)

        # Determine best focus
        best_idx = int(np.argmax(pds_smooth))
        best_z_um = zs_um[best_idx]

        return zs_um, pds_smooth, best_z_um
        