import struct
import numpy as np

def data_to_bytes(x: int, y:int, aux_flag: bool=False) -> bytes:
    '''
    Convert 12-bit X and Y coordinates into a 4-byte packet.

    For X: shift left by 2 bits, add control bits '10' (MSB of the high byte).
    Optionally sets an auxilary flag bit in the low byte.
    For Y: shift left by 2 bits, split into high and low 6-bit values.

    Args:
        x: Raw X coordinate (0 to 4095).
        y: Raw Y coordinate (0 to 4095).
        aux_flag: If True, sets the auxiliary flag bit in X low byte.

    Returns:
        A 4-byte packet: [X_H, X_L, Y_H, Y_L]
    
    Raises:
        ValueError: If x or y are out of range (0 to 4095).
    '''
    if not (0 <= x <= 4095) or not (0 <= y <= 4095):
        raise ValueError("X in Y vrednosti morajo biti med 0 in 4095.")
    
    # Encode X
    a = np.uint16(x) << 2  # Shift X left by 2 bits
    x_low = (np.uint8(a & 0xFF)) >> 2 # Low byte of X
    if aux_flag:
        x_low |= 0b01000000  # Set auxiliary flag bit
    x_high = (np.uint8(a >> 8) & 0xFF) | 0b10000000  # High byte of X with control bits

    # Encode Y
    b = np.uint16(y) << 2  # Shift Y left by 2 bits
    y_low = (np.uint8(b & 0xFF)) >> 2  # Low byte of Y
    y_high = (np.uint8(b >> 8) & 0xFF)  # High byte of Y

    return bytes([x_high, x_low, y_high, y_low])

def create_stepper_packet(steps: int, frequency_hz: int, direction: int) -> bytes:
    '''
    Build a 4-byte packet for a single stepper (Z-axis) command.
    
    Packet layout:
        Byte 1: '11' control bits + upper 6 bits of steps
        Byte 2: lower 6 bits of steps
        Byte 3: bits 7-6 = 00, bit 5 = direction, bits 4-0 = upper 5 bits of frequency_field
        Byte 4: lower 6 bits of frequency_field

    Args:
        steps: Number of steps (0 to 4095).
        frequency_hz: Desired frequency (Hz), must be a multiple of 24.
        direction: 0 for UP, 1 for DOWN.
    
    Returns:
        A 4-byte packet.

    Raises:
        ValueError: If parameters are out of allowed ranges.
    '''
    if not (0 <= steps <= 0xFFF):
        raise ValueError('Število korakov mora biti med 0 in 4095.')
    if frequency_hz % 24 != 0:
        raise ValueError('Frekvenca mora biti večkratnik 24.')
    
    freq_field = frequency_hz // 24
    if not (1 <= freq_field <= 0x7FF):
        raise ValueError('Frekvenčno polje mora biti med 1 in 2047 (24 Hz do 49.968 kHz).')
    
    dir_bit = 1 if direction else 0

    byte1 = 0b11000000 | ((steps >> 6) & 0b00111111)  # '11' + upper 6 bits of steps
    byte2 = steps & 0b00111111  # Lower 6 bits of steps
    byte3 = ((dir_bit & 0b00000001) << 5) | ((freq_field >> 6) & 0b00011111)  # Direction + upper 5 bits of frequency
    byte4 = freq_field & 0b00111111  # Lower 6 bits of frequency
    return bytes([byte1, byte2, byte3, byte4])

def create_zscan_packet(neg_limit: int,
                        pos_limit: int,
                        step_increment: int,
                        frequency_hz: int) -> bytes:
    '''
    Build a 9-byte packet to request a single-shot Z-scan from Arduino.
    
    Packet format (big-endian):
        Byte 0: 0x40 control
        Bytes 1-2: neg_limit (int16)
        Bytes 3-4: pos_limit (int16)
        Bytes 5-6: step_increment (uint16)
        Bytes 7-8: frequency_field = frequency_hz // 24 (uint16)

    Args:
        neg_limit: Negative raw limit (<= 0).
        pos_limit: Positive raw limit (>= 0).
        step_increment: Step size (1 to 4095).
        frequency_hz: Desired frequency (Hz), must be a multiple of 24.

    Returns:
        A 9-byte packet.

    Raises:
        ValueError: If parameters are out of allowed ranges.
    '''
    if neg_limit > 0 or pos_limit < 0:
        raise ValueError('Negativna meja mora biti <= 0, pozitivna meja >= 0.')
    if not (1 <= step_increment <= 0xFFF):
        raise ValueError('Korak mora biti med 1 in 4095.')
    if frequency_hz % 24 != 0:
        raise ValueError('Frekvenca mora biti večkratnik 24.')
    
    freq_field = frequency_hz // 24
    if not (1 <= freq_field <= 0x7FF):
        raise ValueError('Frekvenčno polje mora biti med 1 in 2047 (24 Hz do 49.968 kHz).')
    
    control = 0x40  # Control byte

    # Pack as: >Bh h H H
    # B = control (0x40)
    # h = signed 16-bit integer (neg_limit, pos_limit)
    # H = unsigned 16-bit integer (step_increment, frequency_field)
    return struct.pack(
        '>BhhHH',
        control,
        neg_limit,
        pos_limit,
        step_increment,
        freq_field
    )

def create_autofocus_packet(neg_limit: int,
                            pos_limit: int,
                            step_increment: int,
                            frequency_hz: int) -> bytes:
    '''
    Build a 9-byte packet to perform an autofocus scan.

    Packet format identical to create_zscan_packet, except control=0x41.

    Args:
        neg_limit: Negative raw limit (<= 0).
        pos_limit: Positive raw limit (>= 0).
        step_increment: Step size (1 to 4095).
        frequency_hz: Desired frequency (Hz), must be a multiple of 24.
    Returns:
        A 9-byte packet.
    Raises:
        ValueError: If parameters are out of allowed ranges.
    '''
    if neg_limit > 0 or pos_limit < 0:
        raise ValueError('Negativna meja mora biti <= 0, pozitivna meja >= 0.')
    if not (1 <= step_increment <= 0xFFF):
        raise ValueError('Korak mora biti med 1 in 4095.')
    if frequency_hz % 24 != 0:
        raise ValueError('Frekvenca mora biti večkratnik 24.')
    freq_field = frequency_hz // 24
    if not (1 <= freq_field <= 0x7FF):
        raise ValueError('Frekvenčno polje mora biti med 1 in 2047 (24 Hz do 49.968 kHz).')
    control = 0x41  # Control byte for autofocus
    return struct.pack(
        '>BhhHH',
        control,
        neg_limit,
        pos_limit,
        step_increment,
        freq_field
    )

def create_galvo_scan_packet(x_neg: int,
                             x_pos: int,
                             y_neg: int,
                             y_pos: int,
                             increment: int) -> bytes:
    '''
    Build an 11-byte packet for a full 2D galvo raster scan.
    
    Packet format:
        Byte 0: 0x42 control
        Bytes 1-2: X negative (int16)
        Bytes 3-4: X positive (int16)
        Bytes 5-6: Y negative (int16)
        Bytes 7-8: Y positive (int16)
        Bytes 9-10: Increment (uint16)

    Args:
        x_neg: Negative X offset (<= 0).
        x_pos: Positive X offset (>= 0).
        y_neg: Negative Y offset (<= 0).
        y_pos: Positive Y offset (>= 0).
        increment: Step size (1 to 4095).
    
    Returns:
        A 11-byte packet.

    Raises:
        ValueError: If parameters are out of allowed ranges.
    '''
    if x_neg > 0 or x_pos < 0:
        raise ValueError('Negativna X meja mora biti <= 0, pozitivna X meja >= 0.')
    if y_neg > 0 or y_pos < 0:
        raise ValueError('Negativna Y meja mora biti <= 0, pozitivna Y meja >= 0.')
    if not (1 <= increment <= 0xFFF):
        raise ValueError('Korak mora biti med 1 in 4095.')
    
    control = 0x42  # Control byte for galvo scan
    return struct.pack(
        '>BhhhhH',
        control,
        x_neg,
        x_pos,
        y_neg,
        y_pos,
        increment
    )

def create_3d_layers_packet(x_neg: int,
                            x_pos: int,
                            y_neg: int,
                            y_pos: int,
                            xy_inc: int,
                            z_neg: int,
                            z_pos: int,
                            z_inc: int,
                            frequency_hz: int) -> bytes:
    '''
    Build a 19 byte packet for a layered 3D scan.
    Packet format:
        Byte 0: 0x43 control
        Bytes 1-2: X negative (int16)
        Bytes 3-4: X positive (int16)
        Bytes 5-6: Y negative (int16)
        Bytes 7-8: Y positive (int16)
        Bytes 9-10: XY increment (uint16)
        Bytes 11-12: Z negative (int16)
        Bytes 13-14: Z positive (int16)
        Bytes 15-16: Z increment (uint16)
        Bytes 17-18: Frequency field = frequency_hz // 24 (uint16)
    
    Args:
        x_neg: Negative X offset (<= 0).
        x_pos: Positive X offset (>= 0).
        y_neg: Negative Y offset (<= 0).
        y_pos: Positive Y offset (>= 0).
        xy_inc: XY step size (1 to 4095).
        z_neg: Negative Z offset (<= 0).
        z_pos: Positive Z offset (>= 0).
        z_inc: Z step size (1 to 4095).
        frequency_hz: Desired frequency (Hz), must be a multiple of 24.

    Returns:
        A 19-byte packet.

    Raises:
        ValueError: If parameters are out of allowed ranges.
    '''
    if x_neg > 0 or x_pos < 0:
        raise ValueError('Negativna X meja mora biti <= 0, pozitivna X meja >= 0.')
    if y_neg > 0 or y_pos < 0:
        raise ValueError('Negativna Y meja mora biti <= 0, pozitivna Y meja >= 0.')
    if not (1 <= xy_inc <= 0xFFF):
        raise ValueError('XY korak mora biti med 1 in 4095.')
    if z_neg > 0 or z_pos < 0:
        raise ValueError('Negativna Z meja mora biti <= 0, pozitivna Z meja >= 0.')
    if not (1 <= z_inc <= 0xFFF):
        raise ValueError('Z korak mora biti med 1 in 4095.')
    if frequency_hz % 24 != 0:
        raise ValueError('Frekvenca mora biti večkratnik 24.')
    freq_field = frequency_hz // 24
    if not (1 <= freq_field <= 0x7FF):
        raise ValueError('Frekvenčno polje mora biti med 1 in 2047 (24 Hz do 49.968 kHz).')
    control = 0x43  # Control byte for 3D layers scan

    return struct.pack(
        '>BhhhhHhhHH',
        control,
        x_neg,
        x_pos,
        y_neg,
        y_pos,
        xy_inc,
        z_neg,
        z_pos,
        z_inc,
        freq_field
    )

def create_show_area_packet(x_neg: int,
                            x_pos: int,
                            y_neg: int,
                            y_pos: int,
                            increment: int=1,
                            loops: int=10) -> bytes:
    '''
    Build a packet to outline a rectangular area via galvo loop.

    Packet format:
        Byte 0: 0x44 control
        Bytes 1-2: X negative (int16)
        Bytes 3-4: X positive (int16)
        Bytes 5-6: Y negative (int16)
        Bytes 7-8: Y positive (int16)
        Bytes 9-10: Increment (uint16)
        Bytes 11-12: Loops (uint16)

    Args:
        x_neg: Negative X offset (<= 0).
        x_pos: Positive X offset (>= 0).
        y_neg: Negative Y offset (<= 0).
        y_pos: Positive Y offset (>= 0).
        increment: Step size (1 to 4095).
        loops: Number of loops (1 to 65535).

    Returns:
        A 13-byte packet.

    Raises:
        ValueError: If parameters are out of allowed ranges.
    '''
    if x_neg > 0 or x_pos < 0:
        raise ValueError('Negativna X meja mora biti <= 0, pozitivna X meja >= 0.')
    if y_neg > 0 or y_pos < 0:
        raise ValueError('Negativna Y meja mora biti <= 0, pozitivna Y meja >= 0.')
    if not (1 <= increment <= 0xFFF):
        raise ValueError('Korak mora biti med 1 in 4095.')
    if not (1 <= loops <= 0xFFFF):
        raise ValueError('Število zank mora biti med 1 in 65535.')
    
    control = 0x44
    return struct.pack(
        '>BhhhhHH',
        control,
        x_neg,
        x_pos,
        y_neg,
        y_pos,
        increment,
        loops
    )