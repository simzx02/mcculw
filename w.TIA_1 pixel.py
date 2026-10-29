import os
os.add_dll_directory(r"C:\Program Files (x86)\Measurement Computing\DAQ")

from mcculw import ul
from mcculw.enums import ULRange
import time
import csv
from datetime import datetime
import statistics
import serial

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets, QtGui


# --------------------------
# USB-1808X 설정 Config
# --------------------------
board_num = 0
ai_channel = 7          # CH7H read analog input ch 7
range_val = ULRange.BIP10VOLTS #from -10v to +10v

# --------------------------
# Arduino 설정 Config
# --------------------------
arduino_port = "COM3"  #Change this to your Arduino's COM port
arduino_baud = 115200

# --------------------------
# Sequence 설정 Config
# --------------------------
sequence_start_delay = 10.0

high_time = 1.5
low_time = 3.0

ce_delay = 0.5
ce_off_margin = 0.2

median_threshold_uA = 200.0

drain_pin = 34
resistance_ohm = 1000  # 1k ohm, 10k for 10kohm


class DepositionControl:
    def __init__(self):
        self.app = QtWidgets.QApplication([])

        self.win = pg.GraphicsLayoutWidget(title="USB-1808X CH7H Current Measurement")
        self.win.resize(1000, 500)

        self.plot = self.win.addPlot(title="CH7H Current")
        self.plot.setLabel("left", "Current", units="uA")
        self.plot.setLabel("bottom", "Time", units="s")
        self.plot.setYRange(-50, 300)

        self.curve = self.plot.plot(
            pen=pg.mkPen("r", width=1.5),
            name="CH7H current"
        )

        self.times = []
        self.values_uA = []
        self.log_rows = []

        self.start_time = time.perf_counter()
        self.stop_requested = False
        self.stop_reason = ""

        self.arduino = serial.Serial(arduino_port, arduino_baud, timeout=1)
        time.sleep(2)

        #self.shortcut_space = QtWidgets.QShortcut(QtGui.QKeySequence("Space"), self.win)
        self.shortcut_space = QtGui.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key.Key_Space), self.win)
        self.shortcut_space.activated.connect(self.request_stop)

        self.win.show()

        self.initialize_pins()

    def send_pin(self, pin, high):
        cmd = f"{'H' if high else 'L'}{pin}\n"
        self.arduino.write(cmd.encode())

    def initialize_pins(self):
        for pin in range(22, 38):
            self.send_pin(pin, False)

        self.send_pin(25, True)

        print("[INIT] pin25 HIGH, all other pins LOW")

    def request_stop(self):
        self.stop_requested = True
        self.stop_reason = "Manual stop by space key"
        print("[STOP] Space key pressed")

    def read_current_uA(self):
        voltage = ul.v_in(board_num, ai_channel, range_val)
        #current_uA = voltage * 100.0 ##assume resistor = 10k ohm
        #so if 1v = 100uA, then R = 10k ohm, I = V/R = 1/10000 = 0.0001 A = 100 uA
        current_uA = (voltage / resistance_ohm) * 1000000
        return current_uA

    def update_plot_and_log(
        self,
        phase,
        cycle_count,
        pin22_state,
        pin34_state,
        current_uA,
        cycle_median_uA="",
        recent_3_median_avg_uA="",
    ):
        elapsed = time.perf_counter() - self.start_time

        self.times.append(elapsed)
        self.values_uA.append(current_uA)

        if len(self.times) > 500:
            self.times.pop(0)
            self.values_uA.pop(0)

        self.curve.setData(self.times, self.values_uA)

        row = {
            "time_s": elapsed,
            "threshold_uA": median_threshold_uA,
            "drain_pin": drain_pin,
            "pin34": int(pin34_state),
            "cycle_count": cycle_count,
            "phase": phase,
            "pin22": int(pin22_state),
            "CH7H_current_uA": current_uA,
            "cycle_median_uA": cycle_median_uA,
            "recent_3_median_avg_uA": recent_3_median_avg_uA,
            "stop_reason": "",
        }

        self.log_rows.append(row)

        QtWidgets.QApplication.processEvents()

    def wait_with_measurement(
        self,
        duration,
        phase,
        cycle_count,
        pin22_state=False,
        pin34_state=False,
        collect=False,
    ):
        t0 = time.perf_counter()
        collected_values_uA = []

        while time.perf_counter() - t0 < duration:
            if self.stop_requested:
                break

            current_uA = self.read_current_uA()

            if collect:
                collected_values_uA.append(current_uA)

            self.update_plot_and_log(
                phase=phase,
                cycle_count=cycle_count,
                pin22_state=pin22_state,
                pin34_state=pin34_state,
                current_uA=current_uA,
            )

            time.sleep(0.05)

        return collected_values_uA

    def run_deposition_sequence(self):
        recent_medians_uA = []
        cycle_count = 0

        print("\n[SEQUENCE] START")
        print(f"Drain pin fixed: {drain_pin}")
        print(f"Threshold: {median_threshold_uA:.1f} uA")
        print(f"Cycle: pin{drain_pin} HIGH {high_time} s / LOW {low_time} s")

        while not self.stop_requested:
            cycle_count += 1

            # --------------------------
            # Deposition: pin34 HIGH 1.5 s
            # --------------------------
            self.send_pin(drain_pin, True)

            print(
                f"[CYCLE {cycle_count}] "
                f"pin{drain_pin} HIGH for {high_time} s"
            )

            self.wait_with_measurement(
                duration=high_time,
                phase="DEPOSITION_1P5S",
                cycle_count=cycle_count,
                pin22_state=False,
                pin34_state=True,
                collect=False,
            )

            self.send_pin(drain_pin, False)

            if self.stop_requested:
                break

            # --------------------------
            # Rest / Read: pin34 LOW 3.0 s
            # --------------------------
            print(
                f"[CYCLE {cycle_count}] "
                f"pin{drain_pin} LOW for {low_time} s"
            )

            self.wait_with_measurement(
                duration=ce_delay,
                phase="READ_BEFORE_CE",
                cycle_count=cycle_count,
                pin22_state=False,
                pin34_state=False,
                collect=False,
            )

            if self.stop_requested:
                break

            self.send_pin(22, True)

            ce_high_duration = low_time - ce_delay - ce_off_margin

            measured_during_pin22_uA = self.wait_with_measurement(
                duration=ce_high_duration,
                phase="READ_PIN22_HIGH",
                cycle_count=cycle_count,
                pin22_state=True,
                pin34_state=False,
                collect=True,
            )

            self.send_pin(22, False)

            self.wait_with_measurement(
                duration=ce_off_margin,
                phase="READ_AFTER_CE",
                cycle_count=cycle_count,
                pin22_state=False,
                pin34_state=False,
                collect=False,
            )

            if self.stop_requested:
                break

            if measured_during_pin22_uA:
                med_uA = statistics.median(measured_during_pin22_uA)
                recent_medians_uA.append(med_uA)

                recent_avg_uA = ""

                if len(recent_medians_uA) >= 3:
                    recent_avg_uA = sum(recent_medians_uA[-3:]) / 3

                self.update_plot_and_log(
                    phase="CYCLE_SUMMARY",
                    cycle_count=cycle_count,
                    pin22_state=False,
                    pin34_state=False,
                    current_uA=measured_during_pin22_uA[-1],
                    cycle_median_uA=med_uA,
                    recent_3_median_avg_uA=recent_avg_uA,
                )

                print(f"[CYCLE {cycle_count}] Median = {med_uA:.3f} uA")

                if len(recent_medians_uA) >= 3:
                    print(
                        f"[CYCLE {cycle_count}] "
                        f"Recent 3 median avg = {recent_avg_uA:.3f} uA"
                    )

                    if recent_avg_uA > median_threshold_uA:
                        print(
                            f"[DONE] Recent 3 median avg exceeded "
                            f"{median_threshold_uA:.1f} uA"
                        )
                        self.stop_reason = (
                            f"Completed by recent 3 median avg > "
                            f"{median_threshold_uA:.1f} uA"
                        )
                        break

        self.send_pin(drain_pin, False)
        self.send_pin(22, False)

        return cycle_count, recent_medians_uA

    def finalize_pins(self):
        for pin in range(22, 38):
            self.send_pin(pin, False)

        self.send_pin(25, True)

        print("[FINAL] All pins LOW except pin25 HIGH")

    def save_to_csv(self):
        if not self.log_rows:
            print("[WARN] 저장할 데이터가 없습니다.")
            return

        self.log_rows[-1]["stop_reason"] = self.stop_reason

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"usb1808x_CH7H_pin34_current_uA_{ts}.csv"

        fieldnames = list(self.log_rows[0].keys())

        with open(filename, mode="w", newline="", encoding="utf-8-sig") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.log_rows)

        print(f"[SAVE] CSV 저장 완료: {filename}")
        print(f"[INFO] 종료 사유: {self.stop_reason}")

    def run_sequence(self):
        print("[INFO] 측정 시작")
        print(f"[INFO] Sequence starts after {sequence_start_delay} s")
        print("[INFO] Space bar를 누르면 중간 종료됩니다.")
        print(f"[INFO] Drain pin fixed: {drain_pin}")
        print("[INFO] Cycle: 1.5 s HIGH / 3.0 s LOW")
        print("[INFO] 종료 조건: recent 3 median avg > threshold")
        print(f"[INFO] Threshold = {median_threshold_uA:.1f} uA")

        t0 = time.perf_counter()
        while time.perf_counter() - t0 < sequence_start_delay:
            QtWidgets.QApplication.processEvents()
            time.sleep(0.05)

            if self.stop_requested:
                break

        try:
            if not self.stop_requested:
                self.run_deposition_sequence()

            if not self.stop_requested and self.stop_reason == "":
                self.stop_reason = "Completed pin34 sequence"

        finally:
            self.finalize_pins()

            try:
                self.arduino.close()
            except Exception:
                pass

            self.save_to_csv()
            QtWidgets.QApplication.quit()

    def run(self):
        QtCore.QTimer.singleShot(0, self.run_sequence)
        self.app.exec()


if __name__ == "__main__":
    controller = DepositionControl()
    controller.run()
