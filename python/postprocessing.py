import numpy as np
import matplotlib.pyplot as plt
import plotly.graph_objects as go
import plotly.io as pio

def visualize_3d_layers(path, cmap='viridis', only_three=False, vmin=None, vmax=None):
    '''
    Load a 3D photodiode volume from ``path`` and visualize each Z-layer as a 2D image.

    The file may be a ``.npy`` array or a ``.npz``  archive containing a ``pd_volume`` entry.

    Args:
        path (str): Path to the file containing the 3D photodiode volume.
        cmap (str): Colormap to use for visualization. Default is 'viridis'.
        only_three (bool): If True, only visualize three layers - first, center, and last. Default is False.
        vmin (float): Minimum value for color scaling. Default is None.
        vmax (float): Maximum value for color scaling. Default is None.

    Returns:
        None: Displays the 2D slices of the 3D volume.

    Raises:
        ValueError: If the loaded volume is not 3D or if vmin >= vmax.
    '''
    
    # Load the volume
    if path.endswith('.npz'):
        data = np.load(path)
        volume = data['pd_volume']
    else:
        volume = np.load(path)

    # Validate shape
    if volume.ndim != 3:
        raise ValueError(f'Pričakovano 3D polje! Dejanska oblika: {volume.shape}')
    
    # Determine global colour limits
    if vmin is None or vmax is None:
        vmin = np.min(volume) if vmin is None else vmin
        vmax = np.max(volume) if vmax is None else vmax
    if vmin >= vmax:
        raise ValueError(f'Napačne meje barv: vmin ({vmin}) >= vmax ({vmax})')
    
    nx, ny, nz = volume.shape
    print(f'Naloženo 3D polje {path} z obliko (nx, ny, nz) = ({nx}x{ny}x{nz})')
    print(f'Barvne meje: vmin = {vmin}, vmax = {vmax}')

    # Decide which z-indices to plot
    indices = [0, nz //2, nz - 1] if only_three else range(nz)

    # Plot
    for iz in indices:
        slice_2d = volume[:, :, iz]

        plt.figure(figsize=(8, 6))
        plt.imshow(
            slice_2d.T,
            origin='lower',
            aspect='equal',
            cmap=cmap,
            vmin=vmin,
            vmax=vmax
        )
        plt.colorbar(label='FD meritev')
        plt.xlabel('Galvo X indeks (0..nx-1)')
        plt.ylabel('Galvo Y indeks (0..ny-1)')
        plt.title(f'3D meritev - plast Z = {iz+1}/{nz}')
        plt.tight_layout()
        if only_three:
            plt.show(block=False)
        else:
            plt.show()

    if only_three:
        # Display all prepared figures at once
        plt.show()

def plot_z_profile(volume, x_idx, y_idx, raw_increment=10, z_scale=0.5):
    '''
    Plot the photodiode intensity as a function of Z at a single (X,Y) point.

    Args:
        volume (np.ndarray): 3D photodiode volume.
        x_idx (int): X index of the point to plot.
        y_idx (int): Y index of the point to plot.
        raw_increment (int): Raw steps between successive Z-layers.
        z_scale (float): Scale factor for Z-axis. Default is 0.5.
    '''
    nz = volume.shape[2]
    center_idx = nz // 2
    z = (np.arange(nz) - center_idx) * raw_increment * z_scale
    profile = volume[x_idx, y_idx, :]

    plt.figure(figsize=(8, 6))
    plt.plot(z, profile, marker='.', linestyle='-')
    plt.xlabel('Z (µm)')
    plt.ylabel('Intenziteta (FD)')
    plt.title(f'Z-profil pri (X, Y) = ({x_idx}, {y_idx})')
    plt.axvline(0, linestyle='--', linewidth=0.8) # mark Z=0
    plt.grid()
    plt.tight_layout()
    plt.show()

def plot_cross_section(volume, index, axis='y', raw_increment=10, z_scale=0.5, xy_scale=1.2269):
    '''
    Display an XZ or YZ cross-section through the 3D photodiode volume.

    Args:
        volume (np.ndarray): 3D photodiode volume.
        index (int): Index along the specified axis to plot.
        axis (str): Axis to plot ('x' for XZ, 'y' for YZ).
        raw_increment (int): Raw steps between successive Z-layers.
        z_scale (float): Scale factor for Z-axis. Default is 0.5.
        xy_scale (float): Scale factor for XY axes. Default is 1.2269.

    Returns:
        None: Displays the cross-section plot.        
    '''
    z_max_idx = np.argmax(volume, axis=2)
    nz = volume.shape[2]
    center_idx = nz // 2
    z = (z_max_idx - center_idx) * raw_increment * z_scale

    if axis == 'y':
        nx = z.shape[0]
        x = np.arange(nx) * xy_scale
        # Extraxt the XZ slice at specified Y index
        z_slice = z[:, index]
        title = f'X-Z profil pri Y = {index}'
    else:
        nx = z.shape[1]
        x = np.arange(nx) * xy_scale
        # Extract the YZ slice at specified X index
        z_slice = z[index, :]
        title=f'Y-Z profil pri X = {index}'
    
    plt.figure(figsize=(8, 4))
    plt.plot(x, z_slice, marker='.', linestyle='-')
    plt.xlabel("X (µm)")
    plt.ylabel("Z (µm)")
    plt.title(title)
    plt.grid(True)
    plt.show()

def plot_3d_topography(path, xy_scale, z_scale, raw_increment=10):
    '''
    Plot a 3D topography of the photodiode volume.

    The file may be a ``.npy`` array or a ``.npz`` archive containing a
    ``pd_volume`` entry.

    Args:
        path (str): Path to the file containing the 3D photodiode volume.
        xy_scale (float): Scale factor for XY axes.
        z_scale (float): Scale factor for Z-axis.
        raw_increment (int): Raw steps between successive Z-layers.
    '''
    # Render in browser
    pio.renderers.default = 'browser'

    # Load the 3D volume
    if path.endswith('.npz'):
        data = np.load(path)
        volume = data['pd_volume']
    else:
        volume = np.load(path)
    # Extract filename (without extension) for title
    filename = path.split('/')[-1].split('.')[0]

    # Compute the max-PD layer and convert to physical Z
    z_max_idx = np.argmax(volume, axis=2)
    nz = volume.shape[2]
    center_idx = nz // 2

    z = (z_max_idx - center_idx) * raw_increment * z_scale

    # Create a meshgrid for X and Y coordinates
    nx, ny = z.shape
    x = np.arange(nx) * xy_scale
    y = np.arange(ny) * xy_scale
    X, Y = np.meshgrid(x, y, indexing='ij')

    # Diverging colour map centered on data mid-point
    z_min, z_max = z.min(), z.max()

    surf = go.Surface(
        x=X, y=Y, z=z,
        colorscale='RdBu',
        cmin=z_min, cmax=z_max,
        showscale=True,
        colorbar=dict(title='Z (µm)'),
        hovertemplate='X: %{x:.2f} µm<br>Y: %{y:.2f} µm<br>Z: %{z:.2f} µm<extra></extra>'
        )

    # Slider to adjust vertical exaggeration
    scale_values = np.linspace(0.01, 1.0, 50)
    steps = []
    for s in scale_values:
        steps.append({
            'method': 'restyle',
            'label': f"{s:.2f}",
            'args': [
                {'z': [z * s],
                'cmin': [z_min * s], 'cmax': [z_max * s]},
                {'scene.zaxis.range': [z_min * s, z_max * s]}
            ]
        })

    slider = dict(
        active=len(scale_values) - 1,
        currentvalue={'prefix': 'Vertical scale: '},
        pad={'t': 50},
        steps=steps
    )

    # Assemble the figure
    fig = go.Figure(data=[surf])
    fig.update_layout(
        title=f'3D topografija merjene površine - {filename}',
        width=1000, height=800,
        scene = dict(
            xaxis_title="X (µm)",
            yaxis_title="Y (µm)",
            zaxis_title="Z (µm)",
            zaxis=dict(range=[z_min, z_max]) # [z_min, z_max] range for Z axis
        ),
        sliders=[slider],
    )

    # Save HTML and open
    fig.write_html(f'{filename}.html', auto_open=True)
    fig.show(config={
        'toImageButtonOptions': {
            'format': 'png',
            'filename': '3d_topography',
            'height': 800,
            'width': 1000,
            'scale': 1
        }
    })
