# Direct-drive PANI: Python is controller; Nano is voltage-board client.
# Upload pani_direct_drive_nano.ino. Set COM port and measured baseline below.
# Float at startup -> +0.8 V for 1.5 s -> -0.2 V for 3 s -> repeat.
# Integrate signed current ONLY at -0.2 V; stop at 40 uC or Space.
# Stop sends float; TIA remains continuously connected to CE.
# Dependencies: pyserial pyqtgraph PyQt5 openpyxl mcculw + MCC Windows driver.
import os
os.add_dll_directory(r"C:\Program Files (x86)\Measurement Computing\DAQ")

from collections import deque
from ctypes import POINTER, c_double, cast
from datetime import datetime
import math
import time

import serial
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets
from openpyxl import Workbook

from mcculw import ul
from mcculw.enums import (
    FunctionType, ScanOptions, Status, ULRange
)

# ============================================================
# 1. USB-1808X 설정
# ============================================================

BOARD_NUM = 0
AI_CHANNEL = 7
AI_RANGE = ULRange.BIP10VOLTS

SAMPLE_RATE_HZ = 1000

LOW_LATENCY_SINGLEIO = True
BUFFER_SIZE = 32768

POLL_INTERVAL_S = 0.005
PLOT_INTERVAL_S = 0.05
PLOT_MAX_POINTS = 6000

# ============================================================
# 2. Arduino 설정
# ============================================================

ARDUINO_PORT = "COM3"
ARDUINO_BAUD = 115200

# Nano controls only the external voltage board output D0.
# D0 is a board connector, NOT the Nano RX pin.
# Serial commands: 0 0.8 / 0 -0.2 / 0 f (float).

# ============================================================
# 3. Pulse Sequence 설정
# ============================================================

START_DELAY_S = 10.0

HIGH_TIME_S = 1.5
LOW_TIME_S = 3.0

# ============================================================
# 4. TIA / Baseline / Charge 설정
# ============================================================

FEEDBACK_R_OHM = 390.0

CURRENT_SIGN = 1.0

# 0 V에서 측정한 TIA 출력 전압
# 실제 측정값으로 변경 필요
BASELINE_TIA_V_AT_0V = 0.0

CHARGE_THRESHOLD_UC = 40.0

EARLY_WINDOW_S = 0.5

# ============================================================
# 5. Main Controller
# ============================================================

class DepositionControl:

    def __init__(self):

        self.app = (
            QtWidgets.QApplication.instance()
            or QtWidgets.QApplication([])
        )

        # ----------------------------------------------------
        # Plot 설정
        # ----------------------------------------------------

        self.win = pg.GraphicsLayoutWidget(
            title="PANI | CH7H Current | 1 kHz"
        )

        self.win.resize(1100, 550)

        self.plot = self.win.addPlot(
            title="Baseline-corrected TIA Current"
        )

        self.plot.setLabel(
            "left", "Current", units="uA"
        )

        self.plot.setLabel(
            "bottom", "Time", units="s"
        )

        self.plot.showGrid(x=True, y=True)

        self.curve = self.plot.plot(
            pen=pg.mkPen("r", width=1.4)
        )

        self.times = deque(maxlen=PLOT_MAX_POINTS)
        self.currents = deque(maxlen=PLOT_MAX_POINTS)

        self.last_plot_time = 0.0

        self.win.show()

        # ----------------------------------------------------
        # Space Key 종료
        # ----------------------------------------------------

        shortcut_class = (
            getattr(QtWidgets, "QShortcut", None)
            or QtGui.QShortcut
        )

        self.stop_shortcut = shortcut_class(
            QtGui.QKeySequence("Space"),
            self.win
        )

        self.stop_shortcut.activated.connect(
            self.request_stop
        )

        # ----------------------------------------------------
        # Hardware 변수
        # ----------------------------------------------------

        self.serial_port = None

        self.buffer_handle = None
        self.buffer = None

        self.scan_running = False

        self.scan_epoch = None
        self.actual_rate_hz = None

        self.next_index = 0

        # ----------------------------------------------------
        # Phase 관리
        # ----------------------------------------------------

        self.events = []
        self.event_cursor = 0

        self.cycle_count = 0

        # ----------------------------------------------------
        # Charge 변수
        # ----------------------------------------------------

        self.cycle_charge_uC = 0.0
        self.total_charge_uC = 0.0

        self.cycle_early_uC = 0.0
        self.total_early_uC = 0.0

        self.last_read_index = None
        self.last_read_current_uA = None

        self.log_rows = []

        # ----------------------------------------------------
        # Stop 변수
        # ----------------------------------------------------

        self.stop_requested = False
        self.stop_reason = ""

        self.pulse_high = False

        self.outputs_disabled = False

    # ========================================================
    # Space Key 종료
    # ========================================================

    def request_stop(self):

        if not self.stop_requested:

            self.stop_requested = True

            self.stop_reason = "Manual stop (Space)"

            print("[STOP] Space pressed")

            try:
                self.disable_outputs()
            except Exception as exc:
                print(f"[ERROR] Output float failed: {exc}")

    # ========================================================
    # Nano voltage-board control
    # ========================================================

    def send_output(self, state):
        """Request a voltage or float and wait until Nano confirms application."""
        if state not in ("0.8", "-0.2", "f"):
            raise ValueError(f"Invalid output state: {state}")
        self.serial_port.write(f"0 {state}\n".encode("ascii"))
        self.serial_port.flush()
        reply = self.serial_port.readline().decode("ascii", errors="replace").strip()
        if reply != "OK":
            raise RuntimeError(f"Nano did not acknowledge {state!r}: {reply!r}")

    def init_arduino(self):
        self.serial_port = serial.Serial(
            ARDUINO_PORT, ARDUINO_BAUD, timeout=1, write_timeout=1
        )
        time.sleep(2.0)  # Nano resets when the serial port opens.
        self.serial_port.reset_input_buffer()
        self.send_output("f")  # No applied voltage during the startup delay.
        print("[INIT] Direct-drive output floating; TIA continuously connected")

    def disable_outputs(self):
        if self.outputs_disabled or self.serial_port is None:
            return
        self.send_output("f")
        # Mark disabled only after acknowledgement, allowing retry on failure.
        self.outputs_disabled = True
        print("[FINAL] Direct-drive output floating / Hi-Z")

    # ========================================================
    # USB-1808X Continuous Scan
    # ========================================================

    def start_scan(self):

        self.buffer_handle = (
            ul.scaled_win_buf_alloc(BUFFER_SIZE)
        )

        if not self.buffer_handle:

            raise RuntimeError(
                "DAQ scan buffer allocation failed"
            )

        self.buffer = cast(
            self.buffer_handle,
            POINTER(c_double)
        )

        options = (
            ScanOptions.BACKGROUND
            | ScanOptions.CONTINUOUS
            | ScanOptions.SCALEDATA
        )

        if LOW_LATENCY_SINGLEIO:

            options |= ScanOptions.SINGLEIO

        start_before = time.perf_counter()

        rate = ul.a_in_scan(
            BOARD_NUM,
            AI_CHANNEL,
            AI_CHANNEL,
            BUFFER_SIZE,
            SAMPLE_RATE_HZ,
            AI_RANGE,
            self.buffer_handle,
            options
        )

        start_after = time.perf_counter()

        self.scan_running = True

        self.actual_rate_hz = float(rate)

        if self.actual_rate_hz <= 0:

            raise RuntimeError(
                f"Invalid DAQ scan rate: {rate}"
            )

        self.scan_epoch = (
            start_before + start_after
        ) / 2.0

        self.next_index = 0

        self.events = [{
            "index": 0,
            "phase": "IDLE",
            "cycle": 0,
            "pin_high": False,
            "integrate": False
        }]

        self.event_cursor = 0

        print(
            f"[DAQ] Requested: {SAMPLE_RATE_HZ} Hz"
        )

        print(
            f"[DAQ] Actual: {rate} Hz"
        )

    # ========================================================
    # Phase 전환
    # ========================================================

    def mark_phase(
        self,
        name,
        cycle,
        pin_high,
        integrate
    ):

        before = time.perf_counter()

        self.send_output("0.8" if pin_high else "-0.2")

        after = time.perf_counter()

        self.pulse_high = pin_high

        edge_time = (
            before + after
        ) / 2.0

        index = max(
            0,
            math.ceil(
                (edge_time - self.scan_epoch)
                * self.actual_rate_hz
            )
        )

        index = max(
            index,
            self.next_index
        )

        event = {
            "index": index,
            "phase": name,
            "cycle": cycle,
            "pin_high": pin_high,
            "integrate": integrate
        }

        self.events.append(event)

        print(
            f"[PHASE] {name}, "
            f"cycle={cycle}, "
            f"estimated sample={index}"
        )

        return edge_time

    # ========================================================
    # DAQ Data Processing + Charge Integration
    # ========================================================

    def process_scan_data(self):

        if not self.scan_running:
            return

        status, count, _ = ul.get_status(
            BOARD_NUM,
            FunctionType.AIFUNCTION
        )

        if status != Status.RUNNING:

            raise RuntimeError(
                f"DAQ background scan stopped: {status}"
            )

        if count < self.next_index:

            raise RuntimeError(
                "DAQ scan count reset unexpectedly"
            )

        available = count - self.next_index

        if available > BUFFER_SIZE // 2:

            raise RuntimeError(
                "DAQ buffer overrun suspected"
            )

        if available == 0:
            return

        # ----------------------------------------------------
        # 모든 Sample 처리
        # ----------------------------------------------------

        for index in range(
            self.next_index,
            count
        ):

            # ------------------------------------------------
            # Phase 판별
            # ------------------------------------------------

            while (
                self.event_cursor + 1 < len(self.events)
                and index >= self.events[
                    self.event_cursor + 1
                ]["index"]
            ):

                self.event_cursor += 1
                # Reset when samples reach the edge, not when command is sent.
                self.last_read_index = None
                self.last_read_current_uA = None

            evt = self.events[self.event_cursor]

            # ------------------------------------------------
            # TIA Voltage
            # ------------------------------------------------

            voltage = float(
                self.buffer[index % BUFFER_SIZE]
            )

            # ------------------------------------------------
            # Raw Current (uA)
            # ------------------------------------------------

            raw_uA = (
                CURRENT_SIGN
                * voltage
                / FEEDBACK_R_OHM
                * 1e6
            )

            # ------------------------------------------------
            # Baseline Corrected Current (uA)
            # ------------------------------------------------

            corrected_uA = (
                CURRENT_SIGN
                * (
                    voltage
                    - BASELINE_TIA_V_AT_0V
                )
                / FEEDBACK_R_OHM
                * 1e6
            )

            # ------------------------------------------------
            # Sample Time
            # ------------------------------------------------

            t_abs = (
                self.scan_epoch
                + index / self.actual_rate_hz
            )

            t_elapsed = (
                t_abs - self.scan_epoch
            )

            phase_time = (
                index - evt["index"]
            ) / self.actual_rate_hz

            dt = 0.0
            dq = 0.0
            dq_early = 0.0

            # =================================================
            # -0.2 V 구간에서만 적분
            # =================================================

            if evt["integrate"]:

                # --------------------------------------------
                # 첫 번째 Sample
                # --------------------------------------------

                if self.last_read_index is None:

                    dt = (
                        1.0 / self.actual_rate_hz
                    )

                    dq = corrected_uA * dt

                    interval_start = 0.0

                # --------------------------------------------
                # 이후 Sample: Trapezoidal Integration
                # --------------------------------------------

                else:

                    dt = (
                        index - self.last_read_index
                    ) / self.actual_rate_hz

                    dq = (
                        self.last_read_current_uA
                        + corrected_uA
                    ) * 0.5 * dt

                    interval_start = max(0.0, phase_time - dt)

                # --------------------------------------------
                # 초기 0.5초 적분값 별도 계산
                # --------------------------------------------

                if interval_start < EARLY_WINDOW_S:

                    fraction = min(
                        1.0,
                        max(
                            0.0,
                            (
                                EARLY_WINDOW_S
                                - interval_start
                            ) / dt
                        )
                    )

                    dq_early = dq * fraction

                # --------------------------------------------
                # 이전 Sample 갱신
                # --------------------------------------------

                self.last_read_index = index

                self.last_read_current_uA = (
                    corrected_uA
                )

                # --------------------------------------------
                # Cycle Charge
                # --------------------------------------------

                self.cycle_charge_uC += dq

                # --------------------------------------------
                # Cumulative Charge
                # --------------------------------------------

                self.total_charge_uC += dq

                # --------------------------------------------
                # Initial 0.5 s Charge
                # --------------------------------------------

                self.cycle_early_uC += dq_early

                self.total_early_uC += dq_early

            # =================================================
            # Plot + Excel Logging
            # =================================================

            if evt["phase"] != "IDLE":

                self.times.append(t_elapsed)

                self.currents.append(corrected_uA)

                self.log_rows.append({

                    "time_s": t_elapsed,

                    "cycle": evt["cycle"],

                    "phase": evt["phase"],

                    "phase_time_s":
                        max(0.0, phase_time),

                    "D0_commanded_voltage_V": 0.8 if evt["pin_high"] else -0.2,

                    "TIA_voltage_V": voltage,

                    "baseline_TIA_voltage_V":
                        BASELINE_TIA_V_AT_0V,

                    "raw_current_uA": raw_uA,

                    "corrected_current_uA":
                        corrected_uA,

                    "integration_dt_s": dt,

                    "delta_charge_uC": dq,

                    "cycle_charge_uC":
                        self.cycle_charge_uC,

                    "cumulative_charge_uC":
                        self.total_charge_uC,

                    "cycle_0to0p5s_uC":
                        self.cycle_early_uC,

                    "cumulative_0to0p5s_uC":
                        self.total_early_uC,

                    "threshold_uC":
                        CHARGE_THRESHOLD_UC,

                    "stop_reason": ""
                })

            self.next_index = index + 1

            # =================================================
            # Charge Threshold
            # =================================================

            if evt["integrate"]:

                if (
                    self.total_charge_uC
                    >= CHARGE_THRESHOLD_UC
                ):

                    self.stop_requested = True

                    self.stop_reason = (
                        "Charge threshold reached: "
                        f"{self.total_charge_uC:.4f} uC"
                    )

                    print(
                        f"[DONE] {self.stop_reason}"
                    )

                    self.disable_outputs()

                    break

        # ----------------------------------------------------
        # Plot Update
        # ----------------------------------------------------

        now = time.perf_counter()

        if (
            now - self.last_plot_time
            >= PLOT_INTERVAL_S
        ):

            self.curve.setData(
                list(self.times),
                list(self.currents)
            )

            self.last_plot_time = now

    # ========================================================
    # Phase 유지
    # ========================================================

    def hold_phase(self, duration_s):

        end = (
            time.perf_counter()
            + duration_s
        )

        while (
            not self.stop_requested
            and time.perf_counter() < end
        ):

            self.process_scan_data()

            QtWidgets.QApplication.processEvents()

            if not self.stop_requested:

                time.sleep(POLL_INTERVAL_S)

        if not self.stop_requested:

            self.process_scan_data()

    # ========================================================
    # Deposition Sequence
    # ========================================================

    def run_deposition(self):

        print(
            f"[START] Rf = {FEEDBACK_R_OHM} Ohm"
        )

        print(
            f"[START] Baseline = "
            f"{BASELINE_TIA_V_AT_0V} V"
        )

        print(
            f"[START] Charge Threshold = "
            f"{CHARGE_THRESHOLD_UC} uC"
        )

        while not self.stop_requested:

            self.cycle_count += 1

            self.cycle_charge_uC = 0.0
            self.cycle_early_uC = 0.0

            # ==============================================
            # +0.8 V / 1.5 s
            # ==============================================

            self.mark_phase(
                "DEPOSITION_0P8V",
                self.cycle_count,
                True,
                False
            )

            self.hold_phase(HIGH_TIME_S)

            if self.stop_requested:
                break

            # ==============================================
            # -0.2 V / 3.0 s
            # ==============================================

            self.mark_phase(
                "READ_M0P2V",
                self.cycle_count,
                False,
                True
            )

            self.hold_phase(LOW_TIME_S)

            # ==============================================
            # Cycle Summary
            # ==============================================

            print(
                f"[CYCLE {self.cycle_count}] "
                f"Q(0-0.5s) = "
                f"{self.cycle_early_uC:.4f} uC"
            )

            print(
                f"[CYCLE {self.cycle_count}] "
                f"Q(0-3s) = "
                f"{self.cycle_charge_uC:.4f} uC"
            )

            print(
                f"[CYCLE {self.cycle_count}] "
                f"Total = "
                f"{self.total_charge_uC:.4f} uC"
            )

    # ========================================================
    # Excel 저장
    # ========================================================

    def save_excel(self):

        filename = (
            f"PANI_charge_1kHz_"
            f"{datetime.now():%Y%m%d_%H%M%S}.xlsx"
        )

        wb = Workbook()

        ws = wb.active
        ws.title = "Measurement"

        columns = [

            "time_s",
            "cycle",
            "phase",
            "phase_time_s",

            "D0_commanded_voltage_V",

            "TIA_voltage_V",
            "baseline_TIA_voltage_V",

            "raw_current_uA",
            "corrected_current_uA",

            "integration_dt_s",
            "delta_charge_uC",

            "cycle_charge_uC",
            "cumulative_charge_uC",

            "cycle_0to0p5s_uC",
            "cumulative_0to0p5s_uC",

            "threshold_uC",
            "stop_reason"
        ]

        ws.append(columns)

        if self.log_rows:

            self.log_rows[-1]["stop_reason"] = (
                self.stop_reason
            )

            for row in self.log_rows:

                ws.append([
                    row.get(key)
                    for key in columns
                ])

        ws.freeze_panes = "A2"

        # ----------------------------------------------------
        # Summary Sheet
        # ----------------------------------------------------

        sm = wb.create_sheet("Summary")

        summary = [

            (
                "Requested sample rate (Hz)",
                SAMPLE_RATE_HZ
            ),

            (
                "Low-latency SINGLEIO",
                LOW_LATENCY_SINGLEIO
            ),

            (
                "Actual sample rate (Hz)",
                self.actual_rate_hz
            ),

            (
                "Rf (Ohm)",
                FEEDBACK_R_OHM
            ),

            (
                "0-V TIA baseline output (V)",
                BASELINE_TIA_V_AT_0V
            ),

            (
                "Current sign",
                CURRENT_SIGN
            ),

            (
                "Output control",
                "Nano voltage board D0; float on stop"
            ),

            (
                "HIGH duration (s)",
                HIGH_TIME_S
            ),

            (
                "LOW duration (s)",
                LOW_TIME_S
            ),

            (
                "Threshold (uC)",
                CHARGE_THRESHOLD_UC
            ),

            (
                "Cycles started",
                self.cycle_count
            ),

            (
                "Total signed read charge (uC)",
                self.total_charge_uC
            ),

            (
                "Total first-0.5s charge (uC)",
                self.total_early_uC
            ),

            (
                "Reason",
                self.stop_reason
            ),

            (
                "Timing note",
                "Software-estimated serial transitions; not hardware synchronized"
            ),

            (
                "Baseline note",
                "0-V TIA baseline measured externally"
            )
        ]

        for item in summary:

            sm.append(item)

        wb.save(filename)

        print(f"[SAVE] {filename}")

    # ========================================================
    # Main Sequence
    # ========================================================

    def run_sequence(self):

        try:

            print(
                "[WARNING] Baseline 0.0 V is a "
                "placeholder, not a measurement."
            )

            print("[OUTPUT] Direct drive: +0.8 V / -0.2 V / floating")

            # -----------------------------------------------
            # Arduino 초기화
            # -----------------------------------------------

            self.init_arduino()

            # -----------------------------------------------
            # Initial Delay
            # -----------------------------------------------

            delay_end = (
                time.perf_counter()
                + START_DELAY_S
            )

            while (
                time.perf_counter() < delay_end
                and not self.stop_requested
            ):

                QtWidgets.QApplication.processEvents()

                time.sleep(0.02)

            # -----------------------------------------------
            # DAQ Scan 시작
            # -----------------------------------------------

            if not self.stop_requested:

                self.start_scan()

                self.hold_phase(0.15)

                self.run_deposition()

        except Exception as exc:

            self.stop_requested = True

            self.stop_reason = (
                f"Error: {exc}"
            )

            print(
                f"[ERROR] {exc}"
            )

        finally:

            # -----------------------------------------------
            # Float the external voltage output; TIA remains connected
            # -----------------------------------------------

            try:
                self.disable_outputs()
            except Exception as exc:
                self.stop_reason += f"; output float failed: {exc}"
                print(f"[ERROR] {self.stop_reason}")

            # -----------------------------------------------
            # DAQ 종료
            # -----------------------------------------------

            if self.scan_running:

                try:

                    ul.stop_background(
                        BOARD_NUM,
                        FunctionType.AIFUNCTION
                    )

                except Exception as exc:

                    print(
                        f"[WARN] DAQ stop failed: {exc}"
                    )

                self.scan_running = False

            # -----------------------------------------------
            # DAQ Buffer 해제
            # -----------------------------------------------

            if self.buffer_handle:

                try:

                    ul.win_buf_free(
                        self.buffer_handle
                    )

                except Exception as exc:

                    print(
                        f"[WARN] Buffer free failed: {exc}"
                    )

                self.buffer_handle = None

            # -----------------------------------------------
            # Arduino 종료
            # -----------------------------------------------

            if self.serial_port is not None:

                try:

                    self.serial_port.close()

                except Exception:
                    pass

            # -----------------------------------------------
            # Excel 저장
            # -----------------------------------------------

            try:

                self.save_excel()

            except Exception as exc:

                print(
                    f"[ERROR] Excel save failed: {exc}"
                )

            print(
                f"[EXIT] "
                f"{self.stop_reason or 'Sequence complete'}"
            )

            QtWidgets.QApplication.quit()

    # ========================================================
    # Run GUI
    # ========================================================

    def run(self):

        QtCore.QTimer.singleShot(
            0,
            self.run_sequence
        )

        if hasattr(self.app, "exec"):

            self.app.exec()

        else:

            self.app.exec_()

# ============================================================
# Program Entry
# ============================================================

if __name__ == "__main__":

    DepositionControl().run()
