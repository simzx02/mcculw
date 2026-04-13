from __future__ import absolute_import, division, print_function
from builtins import * # @UnusedWildImport

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from ctypes import cast, POINTER, c_ulong, c_ushort
import time

from mcculw import ul
from mcculw.enums import ScanOptions, FunctionType, AnalogInputMode, ULRange, TimerIdleState
from mcculw.device_info import DaqDeviceInfo

try:
    from console_examples_util import config_first_detected_device
except ImportError:
    from .console_examples_util import config_first_detected_device

def run_example():
    use_device_detection = True
    dev_id_list = []
    board_num = 0
    memhandle = None
    timer_num = 0  # TMR0 (Terminal 16)

    # --- Configuration Parameters ---
    rate = 10000          # 10kHz analog sample rate
    points_per_channel = 1000 
    toggle_freq = 2.0     # 2 Hz Hardware Pulse
    duty_cycle = 0.5      # 50% Duty Cycle
    ai_range = ULRange.BIP5VOLTS 

    accumulated_data = []

    try:
        if use_device_detection:
            config_first_detected_device(board_num, dev_id_list)

        daq_dev_info = DaqDeviceInfo(board_num)
        
        # 1. Start Hardware Pulse Output (Replaces the legacy Timer function)
        # For USB-1808X, pulse_out_start is the correct hardware-timed method.
        # Parameters: board, timer_num, freq, duty_cycle, count(0=continuous), delay, idle_state, options
        print(f"Starting hardware pulse output at {toggle_freq} Hz on TMR0 (Terminal 16)...")
        actual_freq, actual_duty, actual_delay = ul.pulse_out_start(
            board_num, timer_num, toggle_freq, duty_cycle, 0, 0, TimerIdleState.LOW, 0
        )

        # 2. Setup Analog Hardware
        ul.a_input_mode(board_num, AnalogInputMode.DIFFERENTIAL)
        ai_info = daq_dev_info.get_ai_info()
        
        total_count = points_per_channel # Single channel (CH0)

        # 3. Memory Allocation for 18-bit device
        if ai_info.resolution <= 16:
            memhandle = ul.win_buf_alloc(total_count)
            ctypes_array = cast(memhandle, POINTER(c_ushort))
        else:
            memhandle = ul.win_buf_alloc_32(total_count)
            ctypes_array = cast(memhandle, POINTER(c_ulong))

        if not memhandle:
            raise Exception('Error: Failed to allocate memory')

        # 4. Graph Setup
        fig, ax = plt.subplots()
        line, = ax.plot([], [], lw=1, color='purple')
        ax.set_ylim(-0.5, 5.5)
        ax.set_xlim(0, 1000) 
        ax.set_title("Hardware-Timed Pulse Accumulation (USB-1808X)")
        ax.set_ylabel("Voltage (V)")
        ax.set_xlabel("Total Samples")

        def update(frame):
            # No software toggling here; TMR0 handles it autonomously.
            status, curr_count, curr_index = ul.get_status(board_num, FunctionType.AIFUNCTION)
            
            if curr_count > 0:
                # Read latest window from the hardware buffer
                new_samples = [ul.to_eng_units_32(board_num, ai_range, ctypes_array[i]) 
                               for i in range(points_per_channel)]
                
                accumulated_data.extend(new_samples)
                data_len = len(accumulated_data)
                line.set_data(range(data_len), accumulated_data)
                
                # Dynamically expand X-axis
                if data_len > ax.get_xlim()[1]:
                    ax.set_xlim(0, data_len + 1000)
                    fig.canvas.draw() 

            return line,

        # 5. Start Background Analog Scan
        ul.a_in_scan(board_num, 0, 0, total_count, rate, ai_range, memhandle, 
                     ScanOptions.BACKGROUND | ScanOptions.CONTINUOUS)

        ani = FuncAnimation(fig, update, blit=False, interval=50)
        plt.show()

    except KeyboardInterrupt:
        print('\nExperiment stopped by user.')

    except Exception as e:
        print('\nHardware Error:', e)
        
    finally:
        # 6. Critical Cleanup
        # Stop the pulse generator and the analog scan
        ul.pulse_out_stop(board_num, timer_num)
        ul.stop_background(board_num, FunctionType.AIFUNCTION)
        
        if memhandle:
            ul.win_buf_free(memhandle)
        if use_device_detection:
            ul.release_daq_device(board_num)

if __name__ == '__main__':
    run_example()