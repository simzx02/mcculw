from __future__ import absolute_import, division, print_function
from builtins import * # @UnusedWildImport

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from ctypes import cast, POINTER, c_double, c_ulong, c_ushort
import time

from mcculw import ul
from mcculw.enums import ScanOptions, FunctionType, AnalogInputMode, DigitalIODirection, ULRange
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

    # --- Configuration Parameters ---
    rate = 10000          # 10kHz sample rate [cite: 52]
    points_per_channel = 1000 
    toggle_freq = 2      # Hz
    bit_num = 1           # DIO1 (Terminal 46) [cite: 691]
    ai_range = ULRange.BIP5VOLTS # ±5V range [cite: 533, 593]

    # Persistent storage for accumulation
    accumulated_data = []

    try:
        if use_device_detection:
            config_first_detected_device(board_num, dev_id_list)

        daq_dev_info = DaqDeviceInfo(board_num)
        if not daq_dev_info.supports_analog_input or not daq_dev_info.supports_digital_io:
            raise Exception('Error: Device does not support required I/O [cite: 31, 50]')

        print('\nActive DAQ device: ', daq_dev_info.product_name, ' (',
              daq_dev_info.unique_id, ')\n', sep='')

        # 1. Setup Digital Port for MOSFET control
        dio_info = daq_dev_info.get_dio_info()
        port = next((p for p in dio_info.port_info if p.supports_output), None)
        if port.is_port_configurable:
            ul.d_config_port(board_num, port.type, DigitalIODirection.OUT)
        # 2. Setup Analog Hardware
        ul.a_input_mode(board_num, AnalogInputMode.DIFFERENTIAL) 
        ai_info = daq_dev_info.get_ai_info()
        
        low_chan = 0
        high_chan = 0  # Only reading CH0
        num_chans = 1
        total_count = points_per_channel * num_chans

        # 3. Memory Allocation Logic
        scan_options = ScanOptions.BACKGROUND | ScanOptions.CONTINUOUS

        if ai_info.resolution <= 16:
            memhandle = ul.win_buf_alloc(total_count)
            ctypes_array = cast(memhandle, POINTER(c_ushort))
        else:
            # USB-1808X is 18-bit [cite: 51, 69]
            memhandle = ul.win_buf_alloc_32(total_count)
            ctypes_array = cast(memhandle, POINTER(c_ulong))

        if not memhandle:
            raise Exception('Error: Failed to allocate memory [cite: 662]')

        # 4. Graph Setup
        fig, ax = plt.subplots()
        line, = ax.plot([], [], lw=1, color='purple')
        ax.set_ylim(-0.5, 5.5)
        # Initialize X-axis to 0, it will grow dynamically
        ax.set_xlim(0, 1000) 
        ax.set_title("Accumulated LED Branch Voltage (DIFF CH0H-CH0L)")
        ax.set_ylabel("Voltage (V)")
        ax.set_xlabel("Total Samples")

        def update(frame):
            # Software Toggle for DIO1 [cite: 55, 631]
            state = int(time.time() * toggle_freq * 2) % 2
            ul.d_bit_out(board_num, port.type, bit_num, state) 

            # Get latest scan status [cite: 355, 391]
            status, curr_count, curr_index = ul.get_status(board_num, FunctionType.AIFUNCTION)
            
            if curr_count > 0:
                # Read latest window and APPEND to the persistent list
                new_samples = [ul.to_eng_units_32(board_num, ai_range, ctypes_array[i]) 
                               for i in range(points_per_channel)]
                
                accumulated_data.extend(new_samples)
                
                # Update line data and adjust X-axis limits dynamically
                data_len = len(accumulated_data)
                line.set_data(range(data_len), accumulated_data)
                
                if data_len > ax.get_xlim()[1]:
                    ax.set_xlim(0, data_len + 1000)
                    # Necessary to redraw the axis during dynamic growth
                    fig.canvas.draw() 

            return line,

        # 5. Start Background Acquisition [cite: 355, 593]
        ul.a_in_scan(board_num, low_chan, high_chan, total_count,
                     rate, ai_range, memhandle, scan_options)

        ani = FuncAnimation(fig, update, blit=False, interval=50)
        plt.show()

    except KeyboardInterrupt:
            print('\nStopping... Setting bit to 0.')
            ul.d_bit_out(board_num, port.type, bit_num, 0)

    except Exception as e:
        print('\n', e)
    finally:
        # 6. Cleanup [cite: 107, 629]
        ul.stop_background(board_num, FunctionType.AIFUNCTION)
        ul.d_bit_out(board_num, port.type, bit_num, 0)
        if memhandle:
            ul.win_buf_free(memhandle)
        if use_device_detection:
            ul.release_daq_device(board_num)

if __name__ == '__main__':
    run_example()