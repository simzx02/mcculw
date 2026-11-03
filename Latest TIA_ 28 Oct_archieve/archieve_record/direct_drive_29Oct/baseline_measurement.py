"""Measure TIA baseline at commanded 0 V using the direct-drive Nano client.

Upload the accompanying Nano sketch first. Set the COM port below.
The voltage board D0 connects to the working electrode; CE stays on TIA H7.
Copy the printed baseline (volts) into BASELINE_TIA_V_AT_0V in deposition.

It:
1. Commands the Nano’s voltage board to apply 0 V.
2. Waits 1 second, then measures the TIA output for 5 seconds.
3. Calculates the average voltage and prints it.
4. Returns the voltage-board output to floating.

If V data = 0 volt, what is the TIA output? This is the baseline voltage to subtract from future measurements.
"""

import os
os.add_dll_directory(r"C:\Program Files (x86)\Measurement Computing\DAQ")

from mcculw import ul
from mcculw.enums import ULRange

import serial
import time
import statistics


# ============================================================
# 1. USB-1808X 설정
# ============================================================

board_num = 0 
ai_channel = 7  #CH7H, CH7L
range_val = ULRange.BIP10VOLTS


# ============================================================
# 2. Arduino 설정
# ============================================================

arduino_port = "COM3"
arduino_baud = 115200

# No TMUX or gate pins. Hardware pins are defined in the Nano sketch.


# ============================================================
# 3. Baseline 측정 설정
# ============================================================

measurement_time = 5.0   # 5 seconds
sample_interval = 0.001  # Target: 1 ms; software polling, not guaranteed 1 kHz.
settle_time = 1.0        #  current direct-drive settling delay.


# ============================================================
# 4. Arduino Pin 제어
# ============================================================

def send_output(arduino, state):
    # D0 means the external voltage-board output, not Nano RX pin D0.
    # state "0" applies 0 V; state "f" disconnects the output (floating).
    cmd = f"0 {state}\n"

    arduino.write(cmd.encode())
    arduino.flush()

    # The Nano replies after it has applied the requested state.
    reply = arduino.readline().decode("ascii", errors="replace").strip()
    if reply != "OK":
        raise RuntimeError(f"Nano did not acknowledge {state!r}: {reply!r}")


# ============================================================
# 5. Baseline 측정
# ============================================================

def measure_baseline():

    arduino = None
    voltage_values = []

    try:

        # ----------------------------------------------------
        # Arduino 연결
        # ----------------------------------------------------

        arduino = serial.Serial(
            arduino_port,
            arduino_baud,
            timeout=1,
            write_timeout=1
        )

        time.sleep(2.0)

        print("[INIT] Arduino connected")

        # ----------------------------------------------------
        # Direct-drive setup: float first, then apply 0 V.
        # No gate selection or TMUX enable is needed.
        # ----------------------------------------------------
        arduino.reset_input_buffer()
        send_output(arduino, "f")
        print("[INIT] Voltage-board D0 floating")

        send_output(arduino, "0")
        print("[INIT] Voltage-board D0 set to 0 V")
        time.sleep(settle_time)

        print("\n[BASELINE] Measuring TIA output with 0 V applied to pixel")
        print("[BASELINE] Measuring for 5 seconds...")

        start_time = time.perf_counter()

        # ----------------------------------------------------
        # 5초 동안 전압 측정
        # ----------------------------------------------------

        while (
            time.perf_counter() - start_time
            < measurement_time
        ):

            voltage = ul.v_in(
                board_num,
                ai_channel,
                range_val
            )

            voltage_values.append(voltage)

            time.sleep(sample_interval)

        # ----------------------------------------------------
        # 평균 계산
        # ----------------------------------------------------

        if not voltage_values:

            raise RuntimeError(
                "No voltage samples collected"
            )

        baseline_voltage = statistics.fmean(
            voltage_values
        )

        elapsed = (
            time.perf_counter() - start_time
        )

        # ----------------------------------------------------
        # 결과 출력
        # ----------------------------------------------------

        print("\n====================================")
        print("       BASELINE RESULT")
        print("====================================")

        print(
            f"Measurement Time : {elapsed:.3f} s"
        )

        print(
            f"Sample Count     : {len(voltage_values)}"
        )

        print(
            f"Baseline Voltage : "
            f"{baseline_voltage:.6f} V"
        )

        print(
            f"Baseline Voltage : "
            f"{baseline_voltage * 1000:.3f} mV"
        )

        print("====================================")

        print("\n[BASELINE] Measurement completed")

    except Exception as e:

        print(f"[ERROR] {e}")

    finally:

        # ----------------------------------------------------
        # Always float the voltage-board output after measurement.
        # ----------------------------------------------------

        if arduino is not None:

            try:

                send_output(arduino, "f")

                print("\n[FINAL] Voltage-board D0 floating / Hi-Z")

            finally:

                arduino.close()

                print("[FINAL] Arduino disconnected")


# ============================================================
# 6. Program Entry
# ============================================================

if __name__ == "__main__":

    measure_baseline()