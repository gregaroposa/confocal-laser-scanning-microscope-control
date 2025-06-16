import numpy as np
from scipy.signal import savgol_filter

def remove_spikes(pds: np.ndarray,
                  window: int = 7,
                  threshold: float = 3.0) -> np.ndarray:
    '''
    Remove isolated spikes from a 1D photodiode signal by clamping outliers to the local median.

    For each point, compute the median and standard deviation over a sliding window.
    If the absolute difference exceeds threshold * stddev, replace the point with the median.

    Args:
        pds: 1D NumPy array of raw photodiode readings.
        window: Odd integer >= 3 specifying the length of the sliding window.
        threshold: Multiplier of local stddev above which a point is considered a spike.

    Returns:
        A new NumPy array of the same shape as `pds`, with spikes replaced by the local median.
    
    Raises:
        ValueError: If window < 3 or window is even.
    '''
    if window < 3 or window % 2 == 0:
        raise ValueError("Window size must be an odd integer >= 3")
    half = window // 2
    N = pds.shape[0]
    cleaned = pds.astype(float).copy()

    # Pad the signal at both ends to simplify windowing
    pad_left = np.full(half, pds[0], dtype=float)
    pad_right = np.full(half, pds[-1], dtype=float)
    padded = np.concatenate((pad_left, pds, pad_right))

    for i in range(N):
        window_slice = padded[i:i + window]
        med = np.median(window_slice)
        std = window_slice.std()
        if abs(pds[i] - med) > threshold * std:
            cleaned[i] = med
    return cleaned

def savgol_smooth(x: np.ndarray,
                  window_length: int=11,
                  polyorder: int=3) -> np.ndarray:
    '''
    Smooth a 1D signal using Savitzky-Golay filter.

    Applies a convolution-based polynomial smoothing to reduce noise while preserving peaks.

    Args:
        x: 1D NumPy array of data points.
        window_length: Length of the filter window; must be odd and > polyorder.
        polyorder: Order of the polynomial used in filtering; polyorder < window_length.

    Returns:
        Smoothed 1D NumPy array of the same length as input.

    Raises:
        ValueError: If window_length is not odd or less than or equal to polyorder.
    '''
    if window_length % 2 == 0 or window_length <= polyorder:
        raise ValueError("window_length must be an odd integer greater than polyorder")
    
    return savgol_filter(x,
                         window_length=window_length,
                         polyorder=polyorder,
                         mode='interp')
