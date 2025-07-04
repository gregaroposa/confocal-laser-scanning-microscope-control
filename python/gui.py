import tkinter as tk
from tkinter import messagebox, filedialog
import time
import numpy as np
import ttkbootstrap as ttkb
from ttkbootstrap import ttk
import matplotlib.pyplot as plt

from comms import SerialManager
from packets import create_stepper_packet
from scanners.zscan import ZScanScanner, AutofocusScanner
from scanners.galvo import GalvoPointScanner, GalvoRasterScanner, ShowAreaScanner
from scanners.three_d import ThreeDLayerScanner
from postprocessing import (
    visualize_3d_layers,
    plot_z_profile,
    plot_cross_section,
    plot_3d_topography
)

class MicroscopeGUI:
    '''
    Graphical user interface for controlling the confocal microscope.

    Organizes controls into tabs for galvo, manual Z, z-scan, autofocus,
    galvo raster, and 3D layered scan. Uses ttkbootstrap for styling.
    '''

    def __init__(self, port: str='COM4') -> None:
        '''
        Initialize the GUI and establish serial communication.
        Args:
            port: Serial port to connect to the Arduino.
        '''
        self.comms = SerialManager(port)
        self.root = ttkb.Window(themename='minty')
        self.root.title('Krmilna plošča konfokalne mikroskopa')

        # Log window setup
        self.log_win = tk.Toplevel(self.root)
        self.log_win.title('Log')
        self.log_text = tk.scrolledtext.ScrolledText(
            self.log_win, state='disabled', width=80, height=20
        )
        self.log_text.pack(expand=True, fill='both')
        self.log(f'Povezava na {port}.')

        # Conversion scales
        self.XY_SCALE = 1.2269938650306749  # µm per galvo raw step
        self.Z_SCALE = 0.5                  # µm per Z raw step

        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(expand=True, fill='both')

        self._mk_conv_label = self._make_conv_label

        # Build each tab
        self._build_galvo_tab()
        self._build_manual_z_tab()
        self._build_zscan_tab()
        self._build_autofocus_tab()
        self._build_galvo_scan_tab()
        self._build_3d_scan_tab()
        self._build_obdelava_tab()

        self.root.protocol('WM_DELETE_WINDOW', self.on_closing)
        self.log('GUI inicializiran.')

    def log(self, msg:str) -> None:
        '''Append a timestamped message to the log window.'''
        # If the log window was closed, skip logging to avoid Tkinter errors
        if not getattr(self, 'log_text', None):
            return
        try:
            if not self.log_text.winfo_exists():
                return
        except tk.TclError:
            return
        
        ts = time.strftime('%H:%M:%S')
        self.log_text.configure(state='normal')
        self.log_text.insert('end', f'[{ts}] {msg}\n')
        self.log_text.see('end')
        self.log_text.configure(state='disabled')

    def _make_conv_label(self, parent, row, col, var: tk.StringVar, factor: float, unit: str='\u00b5m') -> tk.Label:
        '''
        Create a label next to an entry that shows converted value in microns.
        Updates whenever the StringVar changes.
        '''
        lbl = tk.Label(parent, text='--')
        lbl.grid(row=row, column=col, padx=(2,10), sticky='w')
        def _update(*args):
            try:
                v = float(var.get())
                lbl.config(text=f'{v * factor:.2f} {unit}')
            except Exception:
                lbl.config(text='--')
        var.trace_add('write', _update)
        _update()
        return lbl
    
    def _build_galvo_tab(self) -> None:
        '''Tab for single-point galvo control.'''
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text='Galvo nadzor')

        tk.Label(tab, text='X (raw):').grid(row=0, column=0, padx=5, pady=5, sticky='e')
        self.gx_var = tk.StringVar(value='2047')
        gx_entry = tk.Entry(tab, textvariable=self.gx_var, width=8)
        gx_entry.grid(row=0, column=1, padx=5, pady=5, sticky='w')
        self._mk_conv_label(tab, 0, 2, self.gx_var, self.XY_SCALE)

        tk.Label(tab, text='Y (raw):').grid(row=1, column=0, padx=5, pady=5, sticky='e')
        self.gy_var = tk.StringVar(value='2047')
        gy_entry = tk.Entry(tab, textvariable=self.gy_var, width=8)
        gy_entry.grid(row=1, column=1, padx=5, pady=5, sticky='w')
        self._mk_conv_label(tab, 1, 2, self.gy_var, self.XY_SCALE)

        ttk.Button(tab, text='Pošlji galvo ukaz', command=self._send_galvo).grid(row=2, column=0, columnspan=3, pady=10)

    def _send_galvo(self) -> None:
        '''Callback: send single-point galvo command and display PD reading (confirmation of movement).'''
        try:
            x = int(self.gx_var.get())
            y = int(self.gy_var.get())
        except ValueError:
            messagebox.showerror("Napaka", "Vnesite veljavne celotne vrednosti za X in Y.")
            return
        scanner = GalvoPointScanner(self.comms, x, y)
        pd = scanner.run()
        if pd is None:
            messagebox.showerror("Napaka", "Ni prejete meritve fotodiode.")
        else:
            self.log(f'Galvo premik na X={x}, Y={y}.')
        
    def _build_manual_z_tab(self) -> None:
        '''Tab for manual Z-stepper control.'''
        tab = tk.Frame(self.notebook)
        self.notebook.add(tab, text='Ročni Z nadzor')
        tk.Label(tab, text='Z korak (raw):').grid(row=0, column=0, padx=5, pady=5, sticky='e')
        self.z_steps_var = tk.StringVar(value='1500')
        z_steps = tk.Entry(tab, textvariable=self.z_steps_var, width=8)
        z_steps.grid(row=0, column=1, padx=5, pady=5, sticky='w')
        self._mk_conv_label(tab, 0, 2, self.z_steps_var, self.Z_SCALE)

        tk.Label(tab, text='Frekvenca (Hz):').grid(row=1, column=0, padx=5, pady=5, sticky='e')
        self.z_freq_var = tk.StringVar(value=str(24*60))
        z_freq = tk.Entry(tab, textvariable=self.z_freq_var, width=8)
        z_freq.grid(row=1, column=1, padx=5, pady=5, sticky='w')

        tk.Label(tab, text='Smer').grid(row=2, column=0, padx=5, pady=5, sticky='e')
        self.z_dir_var = tk.IntVar(value=1)
        tk.Radiobutton(tab, text='DOL', variable=self.z_dir_var, value=1).grid(row=2, column=1, padx=5, pady=5, sticky='w')
        tk.Radiobutton(tab, text='GOR', variable=self.z_dir_var, value=0).grid(row=2, column=2, padx=5, pady=5, sticky='w')

        ttk.Button(tab, text='Pošlji Z ukaz', command=self._send_manual_z).grid(row=3, column=0, columnspan=3, pady=10)

    def _send_manual_z(self) -> None:
        '''Callback: send manual Z move and wait for feedback.'''
        try:
            steps = int(self.z_steps_var.get())
            freq = int(self.z_freq_var.get())
            direction = self.z_dir_var.get()
        except ValueError:
            messagebox.showerror("Napaka", "Vnesite veljavne vrednosti za Z korak in frekvenco.")
            return
        try:
            pkt = create_stepper_packet(steps, freq, direction)
        except ValueError as e:
            messagebox.showerror("Napaka", str(e))
            return
        self.comms.write_packet(pkt)
        fb = self.comms.read_feedback()
        if fb:
            self.log(f'Z-stepper premik: {steps} korakov {"DOL" if direction else "GOR"}.')
        else:
            messagebox.showerror("Napaka", "Ni prejete povratne informacije od Arduino.")
        
    def _build_zscan_tab(self) -> None:
        '''Tab for Z-scan with PD measurements.'''
        tab = tk.Frame(self.notebook)
        self.notebook.add(tab, text='Z-sken')

        tk.Label(tab, text='Pozitivna meja (raw ↑)').grid(row=0, column=0, padx=5, pady=5, sticky='e')
        self.zs_neg_var = tk.StringVar(value='1500')
        tk.Entry(tab, textvariable=self.zs_neg_var, width=8).grid(row=0, column=1, padx=5, pady=5, sticky='w')
        self._mk_conv_label(tab, 0, 2, self.zs_neg_var, self.Z_SCALE)

        tk.Label(tab, text='Negativna meja (raw ↓)').grid(row=1, column=0, padx=5, pady=5, sticky='e')
        self.zs_pos_var = tk.StringVar(value='1500')
        tk.Entry(tab, textvariable=self.zs_pos_var, width=8).grid(row=1, column=1, padx=5, pady=5, sticky='w')
        self._mk_conv_label(tab, 1, 2, self.zs_pos_var, self.Z_SCALE)

        tk.Label(tab, text='Korak (raw):').grid(row=2, column=0, padx=5, pady=5, sticky='e')
        self.zs_inc_var = tk.StringVar(value='5')
        tk.Entry(tab, textvariable=self.zs_inc_var, width=8).grid(row=2, column=1, padx=5, pady=5, sticky='w')
        self._mk_conv_label(tab, 2, 2, self.zs_inc_var, self.Z_SCALE)

        tk.Label(tab, text='Frekvenca (Hz):').grid(row=3, column=0, padx=5, pady=5, sticky='e')
        self.zs_freq_var = tk.StringVar(value= str(24*60))
        tk.Entry(tab, textvariable=self.zs_freq_var, width=8).grid(row=3, column=1, padx=5, pady=5, sticky='w')

        # Plot options
        self.zs_plot_var = tk.IntVar(value=1)
        ttk.Checkbutton(tab, text='Prikaži meritve', variable=self.zs_plot_var).grid(row=4, column=0, padx=5, pady=5, sticky='w')

        self.zs_multi_var = tk.IntVar(value=0)
        ttk.Checkbutton(
            tab,
            text='Več zaporednih meritev',
            variable=self.zs_multi_var
        ).grid(row=4, column=1, padx=5, pady=5, sticky='w')
        self.zs_runs_var = tk.StringVar(value='2')
        tk.Entry(tab, textvariable=self.zs_runs_var, width=4).grid(
            row=4, column=2, padx=5, pady=5, sticky='w'
        )
        tk.Label(tab, text='Št. ponovitev').grid(
            row=4, column=3, padx=5, pady=5, sticky='e'
        )

        # Save option
        self.save_zscan = tk.IntVar(value=0)
        ttk.Checkbutton(tab, text='Shrani sken', variable=self.save_zscan).grid(
            row=5, column=0, padx=5, pady=5, sticky='w'
        )
        self.zscan_fname = tk.StringVar(value='zscan')
        tk.Entry(tab, textvariable=self.zscan_fname, width=20).grid(
            row=5, column=1, padx=5, pady=5, sticky='w'
        )

        ttk.Button(tab, text='Zaženi Z-skeniranje', command=self._run_zscan).grid(
            row=6, column=0, columnspan=4, pady=10
        )

    def _run_zscan(self) -> None:
        '''Callback: execute z_scan (single or multi-run) and plot results.'''
        self.log('Začenjam Z-skeniranje...')
        try:
            neg = -abs(int(self.zs_neg_var.get()))
            pos = abs(int(self.zs_pos_var.get()))
            inc = int(self.zs_inc_var.get())
            freq = int(self.zs_freq_var.get())
        except ValueError:
            messagebox.showerror("Napaka", "Vnesite veljavne celotne vrednosti za Z-sken.")
            return
        runs = 1
        if self.zs_multi_var.get():
            try:
                runs = int(self.zs_runs_var.get())
                if runs < 1:
                    raise ValueError("Število ponovitev mora biti vsaj 1.")
            except ValueError:
                messagebox.showerror("Napaka", 
                                     "Vnesite veljavno število ponovitev.")
                return
            
        traces = []
        for i in range(runs):
            scanner = ZScanScanner(self.comms, neg, pos, inc, freq)
            zs, pds = scanner.run()
            if zs.size == 0:
                messagebox.showerror("Napaka", "Ni prejete meritve med Z-sken.")
                return
            traces.append((zs, pds))

        # Optionally save the raw traces
        if self.save_zscan.get():
            fname = self.zscan_fname.get()
            if not fname.endswith('.npz'):
                fname += '.npz'
            np.savez(
                fname,
                zs_runs=[t[0] for t in traces],
                pds_runs=[t[1] for t in traces],
            )
            saved_fname = fname
        else:
            saved_fname = None
        
        if self.zs_plot_var.get():
            plt.figure(figsize=(10,6))
            lines = []
            for idx, (zs, pds) in enumerate(traces, start=1):
                line = plt.plot(zs, pds, marker='.', linestyle='-', label=f'Run {idx}')
                lines.append(line)
                if runs > 1:
                    best = zs[np.argmax(pds)]
                    plt.plot(best, pds.max(), marker='o', color='red', label='_nolegend_')
                    plt.axvline(best, color='red', linestyle='--', label='_nolegend_')
            plt.xlabel('Z (µm)')
            plt.ylabel('PD meritev')
            title = 'Z-sken' + (f' ({runs} ponovitve)' if runs > 1 else '')
            plt.title(title)
            plt.grid()
            if runs > 1:
                plt.legend([f'Run {i}' for i in range(1, runs+1)], bbox_to_anchor=(1,1), loc='upper left')
            plt.tight_layout()
            plt.show()

        if saved_fname:
            self.log(f'Z-skeniranje shranjeno v {saved_fname}')
        else:
            self.log('Z-skeniranje zaključeno.')

    def _build_autofocus_tab(self) -> None:
        '''Tab for autofocus Z-scam'''
        tab = tk.Frame(self.notebook)
        self.notebook.add(tab, text='Autofokus')
        
        tk.Label(tab, text='Pozitivna meja (raw ↑)').grid(row=0, column=0, padx=5, pady=5, sticky='e')
        self.afn = tk.StringVar(value='1500')
        tk.Entry(tab, textvariable=self.afn, width=8).grid(row=0, column=1, padx=5, pady=5, sticky='w')
        self._mk_conv_label(tab, 0, 2, self.afn, self.Z_SCALE)

        tk.Label(tab, text='Negativna meja (raw ↓)').grid(row=1, column=0, padx=5, pady=5, sticky='e')
        self.afp = tk.StringVar(value='1500')
        tk.Entry(tab, textvariable=self.afp, width=8).grid(row=1, column=1, padx=5, pady=5, sticky='w')
        self._mk_conv_label(tab, 1, 2, self.afp, self.Z_SCALE)

        tk.Label(tab, text='Korak (raw):').grid(row=2, column=0, padx=5, pady=5, sticky='e')
        self.af_inc_var = tk.StringVar(value='5')
        tk.Entry(tab, textvariable=self.af_inc_var, width=8).grid(row=2, column=1, padx=5, pady=5, sticky='w')
        self._mk_conv_label(tab, 2, 2, self.af_inc_var, self.Z_SCALE)

        tk.Label(tab, text='Frekvenca (Hz):').grid(row=3, column=0, padx=5, pady=5, sticky='e')
        self.af_freq_var = tk.StringVar(value= str(24*60))
        tk.Entry(tab, textvariable=self.af_freq_var, width=8).grid(row=3, column=1, padx=5, pady=5, sticky='w')

        self.af_plot_var = tk.IntVar(value=1)
        ttk.Checkbutton(tab, text='Prikaži meritve', variable=self.af_plot_var).grid(row=4, column=0, padx=5, pady=5, sticky='w')
        ttk.Button(tab, text='Zaženi Autofokus', command=self._run_autofocus).grid(row=5, column=0, columnspan=3, pady=10)

    def _run_autofocus(self) -> None:
        '''Callback: execute autofocus scan and plot results.'''
        self.log('Začenjam Autofokus...')
        try:
            neg = -abs(int(self.afn.get()))
            pos = abs(int(self.afp.get()))
            inc = int(self.af_inc_var.get())
            freq = int(self.af_freq_var.get())
        except ValueError:
            messagebox.showerror("Napaka", "Vnesite veljavne celotne vrednosti za Autofokus.")
            return
        
        zs, pd, best = AutofocusScanner(self.comms, neg, pos, inc, freq).run()
        if zs.size == 0 or best is None:
            messagebox.showerror("Napaka", "Ni prejete meritve med Autofokusom.")
            return
        
        # Send the stepper move to the best focus position
        raw_steps = int(abs(best / self.Z_SCALE))
        direction = 1 if best > 0 else 0

        freq_to_send = (freq // 24) * 24
        freq_to_send = max(24, min(freq_to_send, 2047 * 24))

        try:
            pkt = create_stepper_packet(raw_steps, freq_to_send, direction)
        except ValueError as e:
            messagebox.showerror("Napaka (stepper)", str(e))
            return
        self.comms.write_packet(pkt)
        fb = self.comms.read_feedback()
        if fb:
            self.log(f'Autofokus zaključen. Najboljši fokus: {best:.2f} µm (koraki {raw_steps}, smer: {"DOL" if direction else "GOR"}).')
        
        if self.af_plot_var.get():
            plt.figure(figsize=(10,6))
            plt.plot(zs, pd, marker='.', linestyle='-')
            plt.axvline(best, color='red', linestyle='--', label=f'Najboljši fokus: {best:.2f} µm')
            plt.xlabel('Z (µm)')
            plt.ylabel('PD meritev')
            plt.title('Autofokus')
            plt.grid()
            plt.legend()
            plt.tight_layout()
            plt.show()

    def _build_galvo_scan_tab(self) -> None:
        '''Tab for full 2D galvo raster scan.'''
        tab = tk.Frame(self.notebook)
        self.notebook.add(tab, text='Galvo skeniranje')

        tk.Label(tab, text='X negativni offset (raw):').grid(row=0, column=0, padx=5, pady=5, sticky='e')
        self.galvo_x_neg = tk.StringVar(value='1000')
        tk.Entry(tab, textvariable=self.galvo_x_neg, width=8).grid(row=0, column=1, padx=5, pady=5, sticky='w')
        self._mk_conv_label(tab, 0, 2, self.galvo_x_neg, self.XY_SCALE)

        tk.Label(tab, text='X pozitivni offset (raw):').grid(row=1, column=0, padx=5, pady=5, sticky='e')
        self.galvo_x_pos = tk.StringVar(value='1000')
        tk.Entry(tab, textvariable=self.galvo_x_pos, width=8).grid(row=1, column=1, padx=5, pady=5, sticky='w')
        self._mk_conv_label(tab, 1, 2, self.galvo_x_pos, self.XY_SCALE)

        tk.Label(tab, text='Y negativni offset (raw):').grid(row=2, column=0, padx=5, pady=5, sticky='e')
        self.galvo_y_neg = tk.StringVar(value='1000')
        tk.Entry(tab, textvariable=self.galvo_y_neg, width=8).grid(row=2, column=1, padx=5, pady=5, sticky='w')
        self._mk_conv_label(tab, 2, 2, self.galvo_y_neg, self.XY_SCALE)

        tk.Label(tab, text='Y pozitivni offset (raw):').grid(row=3, column=0, padx=5, pady=5, sticky='e')
        self.galvo_y_pos = tk.StringVar(value='1000')
        tk.Entry(tab, textvariable=self.galvo_y_pos, width=8).grid(row=3, column=1, padx=5, pady=5, sticky='w')
        self._mk_conv_label(tab, 3, 2, self.galvo_y_pos, self.XY_SCALE)

        tk.Label(tab, text='Korak (raw):').grid(row=4, column=0, padx=5, pady=5, sticky='e')
        self.galvo_inc = tk.StringVar(value='5')
        tk.Entry(tab, textvariable=self.galvo_inc, width=8).grid(row=4, column=1, padx=5, pady=5, sticky='w')
        self._mk_conv_label(tab, 4, 2, self.galvo_inc, self.XY_SCALE)

        # Show area option
        tk.Label(tab, text='Št. zank:').grid(row=5, column=0, padx=5, pady=5, sticky='e')
        self.galvo_loops = tk.StringVar(value='10')
        tk.Entry(tab, textvariable=self.galvo_loops, width=4).grid(row=5, column=1, padx=5, pady=5, sticky='w')
        tk.Button(tab, text='Prikaži območje', command=self._show_area).grid(row=5, column=2, padx=5, pady=5, sticky='w')

        # Save option and scan
        self.save_galvo = tk.IntVar(value=0)
        ttk.Checkbutton(tab, text='Shrani sken', variable=self.save_galvo).grid(row=6, column=0, padx=5, pady=5, sticky='w')
        self.galvo_fname = tk.StringVar(value='galvo_scan')
        tk.Entry(tab, textvariable=self.galvo_fname, width=20).grid(row=6, column=1, padx=5, pady=5, sticky='w')
        ttk.Button(tab, text='Zaženi Galvo skeniranje', command=self._run_galvo_scan).grid(row=7, column=0, columnspan=3, pady=10)

    def _show_area(self) -> None:
        '''Callback: outline a rectangular area with galvo loops.'''
        self.log('Prikazujem območje z galvo...')
        try:
            xn = -abs(int(self.galvo_x_neg.get()))
            xp =  abs(int(self.galvo_x_pos.get()))
            yn = -abs(int(self.galvo_y_neg.get()))
            yp =  abs(int(self.galvo_y_pos.get()))
            loops = int(self.galvo_loops.get())
        except ValueError:
            return messagebox.showerror('Invalid', 'Enter valid integers')
        scanner = ShowAreaScanner(self.comms, xn, xp, yn, yp, int(self.galvo_inc.get()), loops)
        scanner.run()
        self.log(f'Prikaz območja končan.')

        # Center the galvo to (2047, 2047) after showing area
        scanner = GalvoPointScanner(self.comms, 2047, 2047)
        scanner.run()
        self.log('Galvo centriran na (2047, 2047).')

    def _run_galvo_scan(self) -> None:
        """Perform a galvo raster scan and display or save the result."""
        self.log('Začenjam Galvo skeniranje...')
        try:
            xn = -abs(int(self.galvo_x_neg.get()))
            xp =  abs(int(self.galvo_x_pos.get()))
            yn = -abs(int(self.galvo_y_neg.get()))
            yp =  abs(int(self.galvo_y_pos.get()))
            inc = int(self.galvo_inc.get())
        except ValueError:
            return messagebox.showerror('Invalid', 'Enter valid integers')
        x_um, y_um, vol = GalvoRasterScanner(self.comms, xn, xp, yn, yp, inc).run()
        # Center the galvo to (2047, 2047)
        scanner = GalvoPointScanner(self.comms, 2047, 2047)
        scanner.run()
        
        # Discard the first N settling-rows
        discard = 10
        if vol.shape[1] > discard:
            vol = vol[:, discard:]
            # Update y_um to reflect the new shape
            y_um = y_um[discard:]
        
        plt.figure(figsize=(8,6))
        plt.imshow(vol.T, origin='lower', aspect='equal', extent=[x_um[0], x_um[-1], y_um[0], y_um[-1]])
        plt.colorbar(label='PD Measurement')
        plt.xlabel('X (µm)'); plt.ylabel('Y (µm)')
        plt.title('Galvo Scan')
        plt.show()

        fname = self.galvo_fname.get()
        if not fname.endswith('.npz'):
            fname += '.npz'
        if self.save_galvo.get():
            np.savez(fname, pd_volume=vol, x_um=x_um, y_um=y_um)
            self.log(f'Galvo skeniranje shranjeno v {fname}')
        else:
            self.log('Galvo skeniranje zaključeno, ni shranjeno.')

    def _build_3d_scan_tab(self) -> None:
        """Builds the 3D layered scan tab UI."""
        tab = tk.Frame(self.notebook)
        self.notebook.add(tab, text='3D Skeniranje')

        tk.Label(tab, text='X negativni offset (raw):').grid(row=0, column=0, padx=5, pady=5, sticky='e')
        self.xn = tk.StringVar(value='250'); tk.Entry(tab, textvariable=self.xn, width=8).grid(row=0,column=1)
        self._mk_conv_label(tab,0,2,self.xn,self.XY_SCALE)

        tk.Label(tab, text='X pozitivni offset (raw):').grid(row=1,column=0,sticky='e',padx=5,pady=5)
        self.xp = tk.StringVar(value='250'); tk.Entry(tab, textvariable=self.xp, width=8).grid(row=1,column=1)
        self._mk_conv_label(tab,1,2,self.xp,self.XY_SCALE)

        tk.Label(tab, text='Y negativni offset (raw):').grid(row=2,column=0,sticky='e',padx=5,pady=5)
        self.yn = tk.StringVar(value='250'); tk.Entry(tab, textvariable=self.yn, width=8).grid(row=2,column=1)
        self._mk_conv_label(tab,2,2,self.yn,self.XY_SCALE)

        tk.Label(tab, text='Y pozitivni offset (raw):').grid(row=3,column=0,sticky='e',padx=5,pady=5)
        self.yp = tk.StringVar(value='250'); tk.Entry(tab, textvariable=self.yp, width=8).grid(row=3,column=1)
        self._mk_conv_label(tab,3,2,self.yp,self.XY_SCALE)

        tk.Label(tab, text='XY korak (raw):').grid(row=4,column=0,sticky='e',padx=5,pady=5)
        self.xy_inc = tk.StringVar(value='10'); tk.Entry(tab, textvariable=self.xy_inc, width=8).grid(row=4,column=1)
        self._mk_conv_label(tab,4,2,self.xy_inc,self.XY_SCALE)

        tk.Label(tab, text='Z negativni offset (raw):').grid(row=5,column=0,sticky='e',padx=5,pady=5)
        self.zn = tk.StringVar(value='1000'); tk.Entry(tab, textvariable=self.zn, width=8).grid(row=5,column=1)
        self._mk_conv_label(tab,5,2,self.zn,self.Z_SCALE)

        tk.Label(tab, text='Z pozitivni offset (raw):').grid(row=6,column=0,sticky='e',padx=5,pady=5)
        self.zp = tk.StringVar(value='1000'); tk.Entry(tab, textvariable=self.zp, width=8).grid(row=6,column=1)
        self._mk_conv_label(tab,6,2,self.zp,self.Z_SCALE)

        tk.Label(tab, text='Z korak (raw):').grid(row=7,column=0,sticky='e',padx=5,pady=5)
        self.z_inc = tk.StringVar(value='10'); tk.Entry(tab, textvariable=self.z_inc, width=8).grid(row=7,column=1)
        self._mk_conv_label(tab,7,2,self.z_inc,self.Z_SCALE)

        tk.Label(tab, text='Frekvenca (Hz):').grid(row=8,column=0,sticky='e',padx=5,pady=5)
        self.freq = tk.StringVar(value=str(24*60)); tk.Entry(tab, textvariable=self.freq, width=8).grid(row=8,column=1)

        tk.Label(tab, text='Ime datoteke:').grid(row=9,column=0,sticky='e',padx=5,pady=5)
        self.outfile = tk.StringVar(value='3d_scan.npz')
        tk.Entry(tab, textvariable=self.outfile, width=20).grid(row=9,column=1,sticky='w')
        ttk.Button(tab, text='Zaženi 3D skeniranje', command=self._run_3d_scan).grid(row=10, column=0, columnspan=2, pady=10)

    def _run_3d_scan(self) -> None:
        '''Callback: execute 3D layered scan and save results.'''
        self.log('Začenjam 3D skeniranje...')
        try:
            xn = -abs(int(self.xn.get()))
            xp = abs(int(self.xp.get()))
            yn = -abs(int(self.yn.get()))
            yp = abs(int(self.yp.get()))
            xy = int(self.xy_inc.get())
            zn = -abs(int(self.zn.get()))
            zp = abs(int(self.zp.get()))
            zi = int(self.z_inc.get())
            freq = int(self.freq.get())
        except ValueError:
            return messagebox.showerror('Invalid','Enter valid integers')
        x_um, y_um, z_um, vol = ThreeDLayerScanner(self.comms, xn, xp, yn, yp, xy, zn, zp, zi, freq).run()
        
        # Center the galvo to (2047, 2047)
        scanner = GalvoPointScanner(self.comms, 2047, 2047)
        scanner.run()

        # Discard the first N settling-rows in all layers
        discard = 10
        if vol.shape[1] > discard:
            vol = vol[:, discard:, :]
            # Update y_um to reflect the new shape
            y_um = y_um[discard:]

        # Save the results
        fname = self.outfile.get()
        try:
            np.savez(fname, pd_volume=vol, x_um=x_um, y_um=y_um, z_um=z_um)
            self.log(f'3D skeniranje zaključeno. Shranjeno v {fname}')
        except Exception as e:
            self.log(f'Napaka pri shranjevanju 3D skena: {e}')

    def _build_obdelava_tab(self) -> None:
        '''Tab for post-processing and visualization of scans.'''
        tab = tk.Frame(self.notebook)
        self.notebook.add(tab, text='Obdelava')
        
        # File selection and global settings
        ttk.Button(tab, text='Naloži .npz/.npy datoteko', command=self._load_proc_volume).grid(
            row=0, column=0, padx=5, pady=5, sticky='w'
        )
        self.proc_path_var = tk.StringVar(value='')
        tk.Label(tab, textvariable=self.proc_path_var, wraplength=250).grid(
            row=0, column=1, columnspan=2, sticky='w', padx=5
        )

        tk.Label(tab, text='Z korak (raw):').grid(row=1, column=0, sticky='e')
        self.proc_inc = tk.StringVar(value='10')
        tk.Entry(tab, textvariable=self.proc_inc, width=8).grid(
            row=1, column=1, padx=5, sticky='w'
        )

        # 2D layer visualization
        self.layer_three = tk.IntVar(value=0)
        ttk.Checkbutton(tab, text='Samo tri plasti', variable=self.layer_three).grid(
            row=2, column=0, sticky='w', padx=5
        )
        ttk.Button(tab, text='Prikaži plasti', command=self._show_layers).grid(
            row=2, column=1, columnspan=2, pady=5
        )

        # Z profile
        tk.Label(tab, text='X indeks:').grid(row=3, column=0, sticky='e')
        self.zprof_x = tk.StringVar(value='0')
        tk.Entry(tab, textvariable=self.zprof_x, width=6).grid(row=3, column=1, sticky='w')
        tk.Label(tab, text='Y indeks:').grid(row=4, column=0, sticky='e')
        self.zprof_y = tk.StringVar(value='0')
        tk.Entry(tab, textvariable=self.zprof_y, width=6).grid(row=4, column=1, sticky='w')
        ttk.Button(tab, text='Z profil', command=self._show_z_profile).grid(
            row=4, column=2, padx=5, pady=5
        )

        # Cross section
        tk.Label(tab, text='Prerez indeks:').grid(row=5, column=0, sticky='e')
        self.cross_idx = tk.StringVar(value='0')
        tk.Entry(tab, textvariable=self.cross_idx, width=6).grid(row=5, column=1, sticky='w')
        self.cross_axis = tk.StringVar(value='y')
        tk.Radiobutton(tab, text='XZ', variable=self.cross_axis, value='y').grid(row=5, column=2, sticky='w')
        tk.Radiobutton(tab, text='YZ', variable=self.cross_axis, value='x').grid(row=6, column=2, sticky='w')
        ttk.Button(tab, text='Prikaži prerez', command=self._show_cross_section).grid(
            row=6, column=0, columnspan=2, pady=5
        )

        # 3D topography
        ttk.Button(tab, text='3D topografija', command=self._show_topography).grid(
            row=7, column=0, columnspan=3, pady=5
        )

    def _load_proc_volume(self) -> None:
        '''Load a saved volume from disk.'''
        path = filedialog.askopenfilename(
            filetypes=[('NumPy files', '*.npz *.npy')]
        )
        if not path:
            return
        try:
            if path.endswith('.npz'):
                data = np.load(path)
                self.proc_volume = data['pd_volume']
            else:
                self.proc_volume = np.load(path)
        except Exception as e:
            messagebox.showerror('Napaka', f'Napaka pri branju datoteke: {e}')
            return
        self.proc_path = path
        self.proc_path_var.set(path)
        self.log(f'Naložen volumen {path}')

    def _show_layers(self) -> None:
        if not getattr(self, 'proc_path', ''):
            return messagebox.showerror('Napaka', 'Najprej naložite datoteko.')
        only_three = bool(self.layer_three.get())
        visualize_3d_layers(self.proc_path, only_three=only_three)
        self.log('Prikaz plasti končan.')

    def _show_z_profile(self) -> None:
        if getattr(self, 'proc_volume', None) is None:
            return messagebox.showerror('Napaka', 'Najprej naložite datoteko.')
        try:
            x = int(self.zprof_x.get())
            y = int(self.zprof_y.get())
        except ValueError:
            return messagebox.showerror('Napaka', 'Vnesite veljavna indeksa.')
        try:
            inc = int(self.proc_inc.get())
        except ValueError:
            return messagebox.showerror('Napaka', 'Vnesite veljaven Z korak.')
        plot_z_profile(self.proc_volume, x, y, raw_increment=inc, z_scale=self.Z_SCALE)
        self.log('Prikaz Z profila končan.')

    def _show_cross_section(self) -> None:
        if getattr(self, 'proc_volume', None) is None:
            return messagebox.showerror('Napaka', 'Najprej naložite datoteko.')
        try:
            idx = int(self.cross_idx.get())
        except ValueError:
            return messagebox.showerror('Napaka', 'Vnesite veljaven indeks.')
        axis = self.cross_axis.get()
        try:
            inc = int(self.proc_inc.get())
        except ValueError:
            return messagebox.showerror('Napaka', 'Vnesite veljaven Z korak.')
        plot_cross_section(
            self.proc_volume,
            idx,
            axis=axis,
            raw_increment=inc,
            z_scale=self.Z_SCALE,
            xy_scale=self.XY_SCALE,
        )
        self.log(f'Prikaz preseka po {axis.upper()} osi končan.')

    def _show_topography(self) -> None:
        if not getattr(self, 'proc_path', ''):
            return messagebox.showerror('Napaka', 'Najprej naložite datoteko.')
        try:
            inc = int(self.proc_inc.get())
        except ValueError:
            return messagebox.showerror('Napaka', 'Vnesite veljaven Z korak.')
        plot_3d_topography(self.proc_path, self.XY_SCALE, self.Z_SCALE, raw_increment=inc)
        self.log('Prikaz 3D topografije končan.')

    def on_closing(self) -> None:
        '''Handle window close event: clean up serial connection.'''
        self.log('Zapiranje GUI...')
        self.comms.close()
        self.root.destroy()

def main():
    app = MicroscopeGUI()
    app.root.mainloop()

if __name__ == '__main__':
    main()
