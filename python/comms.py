import serial
import time
from typing import Optional

class SerialManager:
    '''
    High-level wrapper for Arduino serial communication using pySerial.

    Encapsulates port setup, packet writes, and common read operations.
    '''
    def __init__(self,
                 port: str,
                 baud: int = 1_000_000,
                 timeout: float = 10.0) -> None:
        '''
        Initialize the serial port and wait for Arduino to reset.

        Args:
            port: Serial port identifier (e.g., 'COM4' or '/dev/ttyUSB0').
            baud: Baud rate for serial communication (defaults to 1_000_000).
            timeout: Read timeout in seconds.
        '''
        self.ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(2) # Wait for Arduino to reset

    def write_packet(self, packet: bytes) -> None:
        '''
        Send a raw packet to Arduino, flushing the input buffer first.
        Args:
            packet: Byte string to send.
        '''
        self.ser.reset_input_buffer()
        self.ser.write(packet)
        self.ser.flush()

    def read_feedback(self) -> Optional[bytes]:
        '''
        Read a 2-byte feedback response from Arduino.
        Returns:
            The raw two bytes if available, otherwise None.
        '''
        data = self.ser.read(2)
        if len(data) < 2:
            print("Timeout: ni odgovora od Arduino.")
            return None
        return data
    
    def read_pd(self, retries: int=3) -> Optional[int]:
        '''
        Read a 14-bit photodiode measurement from Arduino.
        
        The Arduino encodes the 14-bit value into 2 bytes:
        - High byte: MSB=1, bits 6-0 = upper 7 bits of the value
        - Low byte: MSB=0, bits 6-0 = lower 7 bits of the value

        Retrues on timeout or invalid flag bits.

        Args:
            retries: Number of attempts to read the value.

        Returns:
            Decoded integer value, or None if all attempts fail.
        '''
        for attempt in range(1, retries+1):
            hb = self.ser.read(1)
            lb = self.ser.read(1)
            if len(hb) < 1 or len(lb) < 1:
                print(f'Timeout: ni prejete meritve fotodiode (poskus {attempt}/{retries}).')
                continue

            hb_val = hb[0]
            lb_val = lb[0]

            if not (hb_val & 0x80):
                continue  # High byte MSB must be set
            if (lb_val & 0x80):
                continue

            # Combine into 14-bit value
            value = ((hb_val & 0x7F) << 7) | (lb_val & 0x7F)
            return value
        
        print(f'Napaka: ni veljavne meritve fotodiode po {retries} poskusih.')
        return None
    
    def read_count(self) -> Optional[int]:
        '''
        Read a 2-byte unsigned count (big-endian) from serial port.
        
        Commonly used to get the number of data points that follow.
        
        Returns:
            The count as an integer, or None if read fails (insufficient bytes).
        '''
        header = self.ser.read(2)
        if len(header) < 2:
            return None
        return int.from_bytes(header, byteorder='big')
    
    def read_block(self, length: int) -> bytes:
        '''
        Read exactly `length` bytes from the serial port, blocking until done or timeout.

        Args:
            length: Number of bytes to read.

        Returns:
            Byte string of up to `length` bytes (may be shorter on timeout). 
        '''
        buf = bytearray()
        while len(buf) < length:
            chunk = self.ser.read(length - len(buf))
            if not chunk:
                break
            buf.extend(chunk)
        return bytes(buf)
    
    def close(self) -> None:
        '''
        Close the serial port connection.
        '''
        if self.ser.is_open:
            self.ser.close()
            print("Serial port closed.")
        else:
            print("Serial port was already closed.")

    def __enter__(self) -> 'SerialManager':
        '''
        Support context manager entry.
        '''
        return self
    
    def __exit__(self,
                 exc_type,
                 exc_value,
                 traceback) -> None:
        '''
        Exit context: ensure port is closed.
        '''
        self.close()
