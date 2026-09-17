import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft, fftfreq

# ==========================================
# 1. Load CSV Telemetry Data
# ==========================================
csv_filename = "telemetry_log_20260915_104701.csv" # before filtering
# csv_filename = "telemetry_log_20260915_124833.csv" # after filtering
# csv_filename = "telemetry_log_20260915_131052.csv"  # after filtering

try:
    df = pd.read_csv(csv_filename)
except FileNotFoundError:
    print(f"Error: File '{csv_filename}' not found.")
    exit(1)

# ==========================================
# 2. Extract Target Signal & Compute Sampling Rate
# ==========================================
# Data values:
data_name = "Angular_Velocity_rads"  # Target signal
timestamps = df["Timestamp_s"].values
signal = df[data_name].values  # Target signal for vibration/noise analysis

# Calculate actual mean sample rate (Fs) from timestamps
dt = np.diff(timestamps)
mean_dt = np.mean(dt)
Fs = 1.0 / mean_dt  # Sampling frequency in Hz
N = len(signal)      # Total number of sample points

print(f"Loaded {N} samples.")
print(f"Mean sample period: {mean_dt*1000:.3f} ms")
print(f"Calculated sampling frequency (Fs): {Fs:.2f} Hz")
print(f"Nyquist Limit (Fs / 2): {Fs/2.0:.2f} Hz")

# Control input values:
signal_control = df["Control_Input_rads"].values

# ==========================================
# 3. Fast Fourier Transform (FFT) Analysis
# ==========================================
# Detrend signal (remove DC component/mean offset) before FFT
signal_detrended = signal - np.mean(signal)
# Apply Hann window to reduce spectral leakage
window = np.hanning(N)
signal_windowed = signal_detrended * window
# Calculate FFT
fft_output = fft(signal_windowed)
freqs = fftfreq(N, d=1.0/Fs)
# Compute Single-Sided Amplitude Spectrum
half_N = N // 2
positive_freqs = freqs[:half_N]
magnitude = (2.0 / np.sum(window)) * np.abs(fft_output[:half_N])

# Compute FFT for control input signal
signal_control_detrended = signal_control - np.mean(signal_control)
signal_control_windowed = signal_control_detrended * window
fft_output_control = fft(signal_control_windowed)
magnitude_control = (2.0 / np.sum(window)) * np.abs(fft_output_control[:half_N])

# apply IIR filter to data signal - fc = 20 Hz ****************
from scipy.signal import butter, filtfilt, lfilter, spectrogram
order = 1
fc = 10.0
b, a = butter(N=order, Wn=fc, btype='low', fs=Fs)
# Option A: Zero-phase offline filter (No phase shift - Best for post-analysis)
signal_filtered_offline = filtfilt(b, a, signal)
# fft of filtered signal
fft_output_filtered_offline = fft(signal_filtered_offline * window)
magnitude_filtered_offline = (2.0 / np.sum(window)) * np.abs(fft_output_filtered_offline[:half_N])

# Option B: Causal filter (Simulates true real-time microcontroller delay)
signal_filtered_realtime = lfilter(b, a, signal)
fft_output_filtered_realtime = fft(signal_filtered_realtime * window)
magnitude_filtered_realtime = (2.0 / np.sum(window)) * np.abs(fft_output_filtered_realtime[:half_N])

# ==========================================
# 2. Compute Short-Time Fourier Transform (STFT)
# ==========================================
# Window size in samples (e.g., 128 samples @ 100Hz = 1.28-second window)
nperseg = 128  
# Overlap between consecutive windows (75% overlap yields smooth time axis)
noverlap = 96  

freqs, times, Sxx = spectrogram(
    signal, 
    fs=Fs, 
    window='hann', 
    nperseg=nperseg, 
    noverlap=noverlap,
    scaling='density'
)

# Convert power density to Decibels (dB) for better dynamic range visualization
Sxx_db = 10 * np.log10(Sxx + 1e-10)

# ==========================================
# 5. Frequency Graph Visualization (2D Matrix Layout)
# ==========================================
# Constrained layout automatically arranges 2x2 subplots safely without tight_layout crashes
fig, ((ax_t_data, ax_f_data), (ax_t_control, ax_f_control)) = plt.subplots(
    2, 2, figsize=(12, 8), layout="constrained"
)

# 1. Time-Domain Plot (Gyro)
ax_t_data.plot(timestamps, signal, color='lime', label=r'Raw IMU'+ f' ({data_name})')
ax_t_data.plot(timestamps, signal_filtered_realtime, color='orange', label='Filtered (Causal, Real-Time)')
ax_t_data.plot(timestamps, signal_filtered_offline, color='blue', label='Filtered (Offline, Zero-Phase)')
ax_t_data.set_title('Time-Domain ' + data_name, fontweight='bold')
ax_t_data.set_xlabel('Time (s)')
ax_t_data.set_ylabel(data_name)
ax_t_data.grid(True, linestyle=':', alpha=0.7)
ax_t_data.legend(loc='upper right')

# 2. Frequency-Domain Plot (Gyro Spectrum)
ax_f_data.plot(positive_freqs, magnitude, color='crimson', linewidth=1.5, label='Raw IMU Spectrum')
ax_f_data.plot(positive_freqs, magnitude_filtered_realtime, color='orange', linewidth=1.5, label='Filtered (Causal)')
ax_f_data.plot(positive_freqs, magnitude_filtered_offline, color='blue', linewidth=1.5, label='Filtered (Offline)')
ax_f_data.set_title(data_name + ' (FFT Analysis)', fontweight='bold')
ax_f_data.set_xlabel('Frequency (Hz)')
ax_f_data.set_ylabel('Magnitude (rad/s)')
ax_f_data.set_xlim(0, Fs / 2)
ax_f_data.grid(True, linestyle=':', alpha=0.7)
ax_f_data.legend(loc='upper right')

# Highlight dominant peak on Gyro Spectrum
peak_idx = np.argmax(magnitude[1:]) + 1  # Ignore 0 Hz DC term
peak_freq = positive_freqs[peak_idx]
peak_mag = magnitude[peak_idx]
ax_f_data.annotate(
    f'Peak: {peak_freq:.2f} Hz ({peak_mag:.4f} rad/s)',
    xy=(peak_freq, peak_mag),
    xytext=(peak_freq + (Fs * 0.05), peak_mag * 0.9),
    arrowprops=dict(facecolor='black', shrink=0.05, width=1, headwidth=6),
    fontweight='bold'
)

# 3. Time-Domain Plot (Control Input)
ax_t_control.plot(timestamps, signal_control, color='orange', label=r'Control Input ($\omega$)')
ax_t_control.set_title('Time-Domain Control Input', fontweight='bold')
ax_t_control.set_xlabel('Time (s)')
ax_t_control.set_ylabel('Control Input (rad/s)')
ax_t_control.grid(True, linestyle=':', alpha=0.7)
ax_t_control.legend(loc='upper right')

# 4. Frequency-Domain Plot (Control Input Spectrum)
ax_f_control.plot(positive_freqs, magnitude_control, color='blue', linewidth=1.5, label='Control Spectrum')
ax_f_control.set_title('Control Input Frequency (FFT Analysis)', fontweight='bold')
ax_f_control.set_xlabel('Frequency (Hz)')
ax_f_control.set_ylabel('Magnitude (rad/s)')
ax_f_control.set_xlim(0, Fs / 2)
ax_f_control.grid(True, linestyle=':', alpha=0.7)
ax_f_control.legend(loc='upper right')

# Time-Frequency Spectrogram Plot
fig_tf, ax_tf = plt.subplots(figsize=(10, 6), layout="constrained")
mesh = ax_tf.pcolormesh(
    times, 
    freqs, 
    Sxx_db, 
    shading='gouraud', 
    cmap='inferno'
)
# Add Colorbar for Magnitude
cbar = fig.colorbar(mesh, ax=ax_tf)
cbar.set_label('Power Spectral Density (dB/Hz)', fontweight='bold')
# Labels and Limits
ax_tf.set_title('2D Time-Frequency Spectrogram (Gyro Angular Velocity)', fontweight='bold')
ax_tf.set_xlabel('Time (s)', fontweight='bold')
ax_tf.set_ylabel('Frequency (Hz)', fontweight='bold')
ax_tf.set_ylim(0, Fs / 2)  # Bound to Nyquist Limit (50 Hz)
ax_tf.grid(True, linestyle=':', alpha=0.5, color='white')

# Show interactive window directly
plt.show()