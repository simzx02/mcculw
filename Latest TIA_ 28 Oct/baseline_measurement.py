
from mcculw import ul
from mcculw.enums import ULRange

import serial
import time
import statistics


# ============================================================
# 1. USB-1808X 설정
# ============================================================

board_num = 0
ai_channel = 7
range_val = ULRange.BIP10VOLTS


# ============================================================
# 2. Arduino 설정
# ============================================================

arduino_port = "COM14"
arduino_baud = 115200

EN_PIN = 25
SEL_PIN = 34


# ============================================================
# 3. Baseline 측정 설정
# ============================================================

measurement_time = 5.0   # 5 seconds
sample_interval = 0.001  # Target: 1 ms


# ============================================================
# 4. Arduino Pin 제어
# ============================================================

def send_pin(arduino, pin, high):

    cmd = f"{'H' if high else 'L'}{pin}\n"

    arduino.write(cmd.encode())
    arduino.flush()


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
            timeout=1
        )

        time.sleep(2.0)

        print("[INIT] Arduino connected")

        # ----------------------------------------------------
        # 초기 Pin 설정
        # ----------------------------------------------------

        # EN HIGH = MUX OFF
        send_pin(arduino, EN_PIN, True)

        # SEL LOW = SxB (0 V)
        send_pin(arduino, SEL_PIN, False)

        print("[INIT] EN HIGH (MUX OFF)")
        print("[INIT] SEL LOW (SxB = 0 V)")

        # ----------------------------------------------------
        # Baseline 측정 시작
        # ----------------------------------------------------

        # EN LOW = MUX ON
        send_pin(arduino, EN_PIN, False)

        print("\n[BASELINE] MUX ON")
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
        # 반드시 MUX 비활성화
        # ----------------------------------------------------

        if arduino is not None:

            try:

                # EN HIGH = MUX OFF
                send_pin(arduino, EN_PIN, True)

                print("\n[FINAL] EN HIGH (MUX OFF)")

            finally:

                arduino.close()

                print("[FINAL] Arduino disconnected")


# ============================================================
# 6. Program Entry
# ============================================================

if __name__ == "__main__":

    measure_baseline()
