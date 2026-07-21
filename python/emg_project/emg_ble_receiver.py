"""
Receive 20-byte Nykin-EMG BLE notifications, decode ADS1299 Channel 4
samples, apply the sample-rate-adaptive EMG detector and filters, display
synchronized raw and filtered plots, and optionally save both to CSV.

Examples:
    python emg_ble_receiver.py
    python emg_ble_receiver.py --csv recording.csv
    python emg_ble_receiver.py --seconds 10
    python emg_ble_receiver.py --no-plot --csv recording.csv
    python emg_ble_receiver.py --print-samples
"""

from __future__ import annotations

import argparse
import asyncio
import csv
import math
import multiprocessing as mp
import queue as queue_module
import signal
import struct
import sys
import time
from collections import deque
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Optional

from bleak import BleakClient, BleakScanner
from bleak.exc import BleakError

DEVICE_NAME = "Nykin-EMG"
SERVICE_UUID = "c91d0001-4c82-4b1f-ae27-92b8c429fc01"
SAMPLE_CHARACTERISTIC_UUID = "c91d0002-4c82-4b1f-ae27-92b8c429fc01"
STATUS_CHARACTERISTIC_UUID = "c91d0003-4c82-4b1f-ae27-92b8c429fc01"

PROTOCOL_VERSION = 1
PACKET_SIZE_BYTES = 20
MAX_SAMPLES_PER_PACKET = 4
SAMPLE_RATE_HZ = 250.0
HOST_QUEUE_MAX_RECORDS = 8192

# Plotting is intentionally much slower than the 250 Hz sample stream. The
# plot still receives every sample, but the GUI is only repainted at a normal
# screen rate so BLE notifications are not starved by Matplotlib.
# The BLE/filter process publishes a complete rolling snapshot to a separate
# plotting process. Matplotlib therefore cannot block BLE notifications.
PLOT_SNAPSHOT_INTERVAL_S = 0.20
PLOT_PROCESS_QUEUE_SIZE = 1
PLOT_Y_AUTOSCALE_INTERVAL_S = 1.0
PLOT_MAX_DISPLAY_POINTS = 1200
SAMPLE_CONSUMER_WAIT_S = 0.02

# Repeated console writes can further delay the asyncio/BLE event loop. Packet
# loss is still counted exactly, but warnings are summarized at most once per
# interval instead of printing one line for every missing packet.
PACKET_WARNING_INTERVAL_S = 5.0

STATS_PRINT_INTERVAL_S = 5.0
CSV_SETTLE_TIME_S = 2.0

# Sample-rate-adaptive filtering and detector settings.
PLOT_ENVELOPE = False       # False = filtered samples, True = detector envelope
NOTCH_FREQUENCY_HZ = 60.0
NOTCH_Q = 30.0
LOW_PASS_CUTOFF_HZ = 80.0
LOW_PASS_Q = 1.0 / math.sqrt(2.0)
BASELINE_CUTOFF_HZ = 43.0
DECAY_FACTOR = 10.0
THRESHOLD_FORGET_TIME_SECONDS = 40.0


@dataclass(frozen=True)
class SampleRecord:
    timestamp_unix_s: float
    sample_index: int
    raw_adc_count: int
    packet_sequence: int
    filtered_adc_count: float = 0.0
    detector_envelope: float = 0.0
    activity_detected: bool = False


class EmgFilter:
    """Stateful sample-by-sample implementation of the EMG algorithm."""

    def __init__(self) -> None:
        nyquist_frequency = SAMPLE_RATE_HZ / 2.0

        if not 0.0 < NOTCH_FREQUENCY_HZ < nyquist_frequency:
            raise ValueError(
                "NOTCH_FREQUENCY_HZ must be below the Nyquist frequency."
            )
        if not 0.0 < LOW_PASS_CUTOFF_HZ < nyquist_frequency:
            raise ValueError(
                "LOW_PASS_CUTOFF_HZ must be below the Nyquist frequency."
            )
        if not 0.0 < BASELINE_CUTOFF_HZ < nyquist_frequency:
            raise ValueError(
                "BASELINE_CUTOFF_HZ must be below the Nyquist frequency."
            )
        if NOTCH_Q <= 0.0 or LOW_PASS_Q <= 0.0:
            raise ValueError("Filter Q values must be greater than zero.")

        self.moving_average_factor = math.exp(
            -2.0 * math.pi * BASELINE_CUTOFF_HZ / SAMPLE_RATE_HZ
        )
        self.threshold_decay_factor = math.exp(
            -1.0 / (SAMPLE_RATE_HZ * THRESHOLD_FORGET_TIME_SECONDS)
        )

        notch_w0 = 2.0 * math.pi * NOTCH_FREQUENCY_HZ / SAMPLE_RATE_HZ
        notch_alpha = math.sin(notch_w0) / (2.0 * NOTCH_Q)
        notch_a0 = 1.0 + notch_alpha
        self.notch_b0 = 1.0 / notch_a0
        self.notch_b1 = -2.0 * math.cos(notch_w0) / notch_a0
        self.notch_b2 = 1.0 / notch_a0
        self.notch_a1 = -2.0 * math.cos(notch_w0) / notch_a0
        self.notch_a2 = (1.0 - notch_alpha) / notch_a0

        low_w0 = 2.0 * math.pi * LOW_PASS_CUTOFF_HZ / SAMPLE_RATE_HZ
        low_alpha = math.sin(low_w0) / (2.0 * LOW_PASS_Q)
        low_cos_w0 = math.cos(low_w0)
        low_a0 = 1.0 + low_alpha
        self.low_b0 = ((1.0 - low_cos_w0) / 2.0) / low_a0
        self.low_b1 = (1.0 - low_cos_w0) / low_a0
        self.low_b2 = ((1.0 - low_cos_w0) / 2.0) / low_a0
        self.low_a1 = (-2.0 * low_cos_w0) / low_a0
        self.low_a2 = (1.0 - low_alpha) / low_a0

        self.moving_average = 0.0
        self.decay = 0.0
        self.detector_id = 0.0
        self.detection_threshold = 0.0
        self.data_in = 0.0
        self.sample_count = 0

        self.notch_x1 = 0.0
        self.notch_x2 = 0.0
        self.notch_y1 = 0.0
        self.notch_y2 = 0.0

        self.low_x1 = 0.0
        self.low_x2 = 0.0
        self.low_y1 = 0.0
        self.low_y2 = 0.0

    def process_record(self, record: SampleRecord) -> SampleRecord:
        data_in_old = self.data_in
        self.data_in = float(record.raw_adc_count)

        if self.sample_count == 0:
            self.moving_average = self.data_in
        else:
            self.moving_average = (
                self.moving_average_factor * self.moving_average
                + (1.0 - self.moving_average_factor) * self.data_in
            )
            self.detection_threshold = max(
                self.detection_threshold,
                3.0 * abs(self.data_in - data_in_old),
            )
            self.detection_threshold *= self.threshold_decay_factor

        dc_removed = self.data_in - self.moving_average

        # The detector intentionally uses the unfiltered, DC-removed sample.
        detector_data_point = dc_removed

        notch_output = (
            self.notch_b0 * dc_removed
            + self.notch_b1 * self.notch_x1
            + self.notch_b2 * self.notch_x2
            - self.notch_a1 * self.notch_y1
            - self.notch_a2 * self.notch_y2
        )
        self.notch_x2 = self.notch_x1
        self.notch_x1 = dc_removed
        self.notch_y2 = self.notch_y1
        self.notch_y1 = notch_output

        low_pass_output = (
            self.low_b0 * notch_output
            + self.low_b1 * self.low_x1
            + self.low_b2 * self.low_x2
            - self.low_a1 * self.low_y1
            - self.low_a2 * self.low_y2
        )
        self.low_x2 = self.low_x1
        self.low_x1 = notch_output
        self.low_y2 = self.low_y1
        self.low_y1 = low_pass_output

        self.detector_id += abs(detector_data_point) - self.decay * DECAY_FACTOR
        self.decay += 1.0

        if self.detector_id < 0.0:
            self.detector_id = 0.0
            self.decay = 0.0

        activity_detected = self.detector_id > self.detection_threshold
        filtered_data_point = float(activity_detected) * low_pass_output
        detector_envelope = float(activity_detected) * self.detector_id
        self.sample_count += 1

        return replace(
            record,
            filtered_adc_count=filtered_data_point,
            detector_envelope=detector_envelope,
            activity_detected=activity_detected,
        )


@dataclass
class ReceiverStats:
    total_packets: int = 0
    total_samples: int = 0
    missing_packets: int = 0
    malformed_packets: int = 0
    protocol_errors: int = 0
    out_of_order_packets: int = 0
    host_queue_drops: int = 0
    last_sequence: Optional[int] = None
    connection_generation: int = 0
    last_queue_warning_monotonic: float = 0.0
    pending_missing_packet_warning: int = 0
    last_packet_warning_monotonic: float = 0.0

    def begin_connection(self) -> int:
        """Start a new sequence-tracking epoch and return its generation ID."""
        self.connection_generation += 1
        self.last_sequence = None
        return self.connection_generation

    def invalidate_connection(self) -> None:
        """Ignore notification work that was queued by an old connection."""
        self.connection_generation += 1
        self.last_sequence = None


def _plot_limits(values: list[float]) -> tuple[float, float]:
    if not values:
        return (-1.0, 1.0)

    y_min = min(values)
    y_max = max(values)
    y_range = y_max - y_min
    if y_range == 0.0:
        padding = max(1.0, abs(y_max) * 0.05)
    else:
        padding = 0.08 * y_range
    return (y_min - padding, y_max + padding)


def _open_save_dialog() -> Optional[str]:
    from tkinter import Tk, filedialog

    root = Tk()
    root.withdraw()
    try:
        filename = filedialog.asksaveasfilename(
            title="Save EMG data",
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv")],
            initialfile=time.strftime("emg_recording_%Y%m%d_%H%M%S.csv"),
        )
    finally:
        root.destroy()

    return filename or None


def plot_process_main(
    seconds: float,
    snapshot_queue: object,
    command_queue: object,
) -> None:
    """Run the complete Matplotlib GUI in a separate OS process."""
    import matplotlib.pyplot as plt
    import numpy as np
    from matplotlib.widgets import Button

    figure, axes = plt.subplots(2, 1, sharex=True)
    raw_axes = axes[0]
    filtered_axes = axes[1]
    figure.subplots_adjust(bottom=0.14, hspace=0.28)
    figure.suptitle("Nykin-EMG — ADS1299 Channel 4")

    (raw_line,) = raw_axes.plot([], [])
    raw_axes.set_title("Raw / Unfiltered")
    raw_axes.set_ylabel("Raw ADC count")
    raw_axes.grid(True)
    raw_axes.set_xlim(-seconds, 0.0)

    (filtered_line,) = filtered_axes.plot([], [])
    filtered_axes.set_title("Filtered")
    filtered_axes.set_xlabel("Time relative to newest received sample (s)")
    filtered_axes.set_ylabel("Filtered ADC count")
    filtered_axes.grid(True)
    filtered_axes.set_xlim(-seconds, 0.0)

    button_axes = figure.add_axes([0.78, 0.025, 0.18, 0.055])
    save_button = Button(button_axes, "Save CSV")

    def request_save(_event: object) -> None:
        filename = _open_save_dialog()
        if filename is None:
            return
        try:
            command_queue.put_nowait(("save", filename))
        except queue_module.Full:
            print("WARNING: save request queue was full.", file=sys.stderr)

    save_button.on_clicked(request_save)

    closed = False

    def on_close(_event: object) -> None:
        nonlocal closed
        closed = True
        try:
            command_queue.put_nowait(("closed", None))
        except queue_module.Full:
            pass

    figure.canvas.mpl_connect("close_event", on_close)
    plt.show(block=False)

    last_autoscale = 0.0
    latest_snapshot = None

    while not closed and plt.fignum_exists(figure.number):
        # The queue contains only the newest complete rolling snapshot. Drain it
        # so the plot never wastes time rendering stale frames.
        try:
            while True:
                item = snapshot_queue.get_nowait()
                if item is None:
                    closed = True
                    break
                latest_snapshot = item
        except queue_module.Empty:
            pass

        if latest_snapshot is not None and not closed:
            timestamps, raw_values, filtered_values = latest_snapshot

            if timestamps:
                all_timestamps = np.asarray(timestamps, dtype=float)
                all_raw_values = np.asarray(raw_values, dtype=float)
                all_filtered_values = np.asarray(filtered_values, dtype=float)

                display_stride = max(
                    1,
                    math.ceil(len(all_timestamps) / PLOT_MAX_DISPLAY_POINTS),
                )
                selected_indices = np.arange(
                    0, len(all_timestamps), display_stride, dtype=int
                )

                newest_timestamp = all_timestamps[-1]
                x_values = all_timestamps[selected_indices] - newest_timestamp
                raw_display = all_raw_values[selected_indices].copy()
                filtered_display = all_filtered_values[selected_indices].copy()

                # Break the trace at missing-sample gaps rather than drawing a
                # misleading straight line across lost BLE packets. This test
                # is performed before display decimation, so normal decimation
                # is not mistaken for packet loss.
                if all_timestamps.size > 1 and selected_indices.size > 1:
                    source_gaps = np.diff(all_timestamps) > (2.5 / SAMPLE_RATE_HZ)
                    for display_position in range(1, selected_indices.size):
                        previous_source = selected_indices[display_position - 1]
                        current_source = selected_indices[display_position]
                        if np.any(source_gaps[previous_source:current_source]):
                            raw_display[display_position] = np.nan
                            filtered_display[display_position] = np.nan

                raw_line.set_data(x_values, raw_display)
                filtered_line.set_data(x_values, filtered_display)

                now = time.monotonic()
                if now - last_autoscale >= PLOT_Y_AUTOSCALE_INTERVAL_S:
                    last_autoscale = now
                    raw_axes.set_ylim(*_plot_limits([float(v) for v in raw_values]))
                    filtered_axes.set_ylim(
                        *_plot_limits([float(v) for v in filtered_values])
                    )

                raw_axes.set_xlim(-seconds, 0.0)
                filtered_axes.set_xlim(-seconds, 0.0)
                figure.canvas.draw_idle()

            latest_snapshot = None

        # GUI work occurs only in this child process. Any slowdown here can at
        # worst reduce display frame rate; it cannot delay Bleak callbacks.
        plt.pause(0.02)

    if plt.fignum_exists(figure.number):
        plt.close(figure)


class PlotProcessController:
    """Publish synchronized snapshots without ever blocking BLE reception."""

    def __init__(self, seconds: float) -> None:
        self._seconds = seconds
        self._max_points = max(1, int(round(seconds * SAMPLE_RATE_HZ)))
        self._timestamps: deque[float] = deque(maxlen=self._max_points)
        self._raw_values: deque[int] = deque(maxlen=self._max_points)
        self._filtered_values: deque[float] = deque(maxlen=self._max_points)
        self.snapshot_replacements = 0

        context = mp.get_context("spawn")
        self._snapshot_queue = context.Queue(maxsize=PLOT_PROCESS_QUEUE_SIZE)
        self._command_queue = context.Queue(maxsize=8)
        self._process = context.Process(
            target=plot_process_main,
            args=(seconds, self._snapshot_queue, self._command_queue),
            name="NykinEmgPlot",
        )
        self._process.start()

    def add_records(self, records: list[SampleRecord]) -> None:
        for record in records:
            self._timestamps.append(record.timestamp_unix_s)
            self._raw_values.append(record.raw_adc_count)
            self._filtered_values.append(record.filtered_adc_count)

    def publish_latest(self) -> None:
        snapshot = (
            list(self._timestamps),
            list(self._raw_values),
            list(self._filtered_values),
        )

        try:
            self._snapshot_queue.put_nowait(snapshot)
            return
        except queue_module.Full:
            pass

        # Replace an unrendered stale snapshot with the newest state. This is a
        # display-only replacement; no acquisition, filtering, or CSV data is
        # discarded.
        try:
            self._snapshot_queue.get_nowait()
        except queue_module.Empty:
            pass

        try:
            self._snapshot_queue.put_nowait(snapshot)
            self.snapshot_replacements += 1
        except queue_module.Full:
            self.snapshot_replacements += 1

    def poll_commands(self) -> list[tuple[str, Optional[str]]]:
        commands: list[tuple[str, Optional[str]]] = []
        while True:
            try:
                commands.append(self._command_queue.get_nowait())
            except queue_module.Empty:
                break
        return commands

    def is_alive(self) -> bool:
        return self._process.is_alive()

    def close(self) -> None:
        try:
            self._snapshot_queue.put_nowait(None)
        except queue_module.Full:
            try:
                self._snapshot_queue.get_nowait()
            except queue_module.Empty:
                pass
            try:
                self._snapshot_queue.put_nowait(None)
            except queue_module.Full:
                pass

        self._process.join(timeout=2.0)
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=1.0)

        self._snapshot_queue.close()
        self._command_queue.close()


def write_records_to_csv(records: list[SampleRecord], path: Path) -> None:
    if not records:
        print(
            f"No settled samples to save yet. Wait at least "
            f"{CSV_SETTLE_TIME_S:g} seconds after readings begin."
        )
        return

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(
            [
                "timestamp_unix_s",
                "sample_index",
                "raw_adc_count",
                "filtered_adc_count",
                "detector_envelope",
                "activity_detected",
                "packet_sequence",
            ]
        )
        writer.writerows(
            (
                f"{record.timestamp_unix_s:.6f}",
                record.sample_index,
                record.raw_adc_count,
                f"{record.filtered_adc_count:.6f}",
                f"{record.detector_envelope:.6f}",
                int(record.activity_detected),
                record.packet_sequence,
            )
            for record in records
        )

    print(f"Saved {len(records)} settled samples to: {path.resolve()}")


def bounded_float(
    value: str,
    *,
    minimum: float,
    maximum: float,
    label: str,
) -> float:
    try:
        result = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"{label} must be a number") from exc

    if not minimum <= result <= maximum:
        raise argparse.ArgumentTypeError(
            f"{label} must be between {minimum:g} and {maximum:g}"
        )
    return result


def plot_seconds(value: str) -> float:
    return bounded_float(value, minimum=0.5, maximum=3600.0, label="--seconds")


def positive_timeout(value: str) -> float:
    return bounded_float(value, minimum=0.1, maximum=300.0, label="timeout")


def reconnect_delay(value: str) -> float:
    return bounded_float(
        value,
        minimum=0.0,
        maximum=300.0,
        label="--reconnect-delay",
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Connect to Nykin-EMG over BLE, decode 20-byte ADS1299 sample "
            "packets, plot recent data, and optionally record CSV."
        )
    )
    parser.add_argument(
        "--csv",
        type=Path,
        metavar="PATH",
        help=(
            "Write raw and filtered samples, detector output, activity flag, "
            "and packet metadata to this CSV file. Existing files are replaced."
        ),
    )
    parser.add_argument(
        "--seconds",
        type=plot_seconds,
        default=8.0,
        help="Number of recent seconds shown in both synchronized plots (default: 8).",
    )
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="Disable the Matplotlib window; useful for CSV-only recording.",
    )
    parser.add_argument(
        "--print-samples",
        action="store_true",
        help=(
            "Print each sample as sample_index,raw_adc_count,"
            "filtered_adc_count,detector_envelope,activity_detected."
        ),
    )
    parser.add_argument(
        "--scan-timeout",
        type=positive_timeout,
        default=10.0,
        help="Seconds to scan for Nykin-EMG per attempt (default: 10).",
    )
    parser.add_argument(
        "--connect-timeout",
        type=positive_timeout,
        default=30.0,
        help="BLE connection timeout in seconds (default: 30).",
    )
    parser.add_argument(
        "--reconnect-delay",
        type=reconnect_delay,
        default=3.0,
        help="Delay before scanning again after failure/disconnect (default: 3).",
    )
    return parser.parse_args()


def matches_target_device(device: object, advertisement_data: object) -> bool:
    """Match the advertised local name, with device.name as a Windows fallback."""
    local_name = getattr(advertisement_data, "local_name", None)
    device_name = getattr(device, "name", None)
    return local_name == DEVICE_NAME or device_name == DEVICE_NAME


def report_packet_gap(stats: ReceiverStats, sequence: int) -> None:
    """Track a 16-bit sequence with correct 65535 -> 0 wraparound."""
    if stats.last_sequence is None:
        stats.last_sequence = sequence
        return

    expected = (stats.last_sequence + 1) & 0xFFFF
    if sequence != expected:
        forward_distance = (sequence - expected) & 0xFFFF

        if forward_distance < 0x8000:
            stats.missing_packets += forward_distance
            stats.pending_missing_packet_warning += forward_distance

            # Console output is surprisingly expensive on Windows terminals.
            # Summarize bursts of packet loss instead of printing one warning
            # for every single gap. The exact total remains in ReceiverStats.
            now = time.monotonic()
            if (
                now - stats.last_packet_warning_monotonic
                >= PACKET_WARNING_INTERVAL_S
            ):
                print(
                    "WARNING: BLE packet loss: "
                    f"{stats.pending_missing_packet_warning} packet(s) missing "
                    f"since the previous warning; total={stats.missing_packets}. "
                    f"Latest expected sequence {expected}, received {sequence}",
                    file=sys.stderr,
                )
                stats.pending_missing_packet_warning = 0
                stats.last_packet_warning_monotonic = now
        else:
            # BLE notifications should remain ordered. A backwards jump is
            # reported separately instead of being counted as ~65535 losses.
            stats.out_of_order_packets += 1
            print(
                f"WARNING: duplicate/out-of-order packet; expected sequence "
                f"{expected}, received {sequence}",
                file=sys.stderr,
            )

    stats.last_sequence = sequence


def decode_and_enqueue_packet(
    packet: bytes,
    generation: int,
    sample_queue: asyncio.Queue[SampleRecord],
    stats: ReceiverStats,
) -> None:
    """Decode one notification on the asyncio thread and enqueue its samples."""
    if generation != stats.connection_generation:
        return

    if len(packet) != PACKET_SIZE_BYTES:
        stats.malformed_packets += 1
        print(
            f"WARNING: notification length was {len(packet)} bytes; "
            f"expected {PACKET_SIZE_BYTES}",
            file=sys.stderr,
        )
        return

    version = packet[0]
    sample_count = packet[1]

    if version != PROTOCOL_VERSION:
        stats.protocol_errors += 1
        print(
            f"WARNING: unsupported protocol version {version}; "
            f"expected {PROTOCOL_VERSION}",
            file=sys.stderr,
        )
        return

    if sample_count > MAX_SAMPLES_PER_PACKET:
        stats.malformed_packets += 1
        print(
            f"WARNING: invalid sample count {sample_count}; maximum is "
            f"{MAX_SAMPLES_PER_PACKET}",
            file=sys.stderr,
        )
        return

    (sequence,) = struct.unpack_from("<H", packet, 2)
    report_packet_gap(stats, sequence)
    stats.total_packets += 1

    if sample_count == 0:
        return

    samples = struct.unpack_from(f"<{sample_count}i", packet, 4)
    packet_receive_time = time.time()

    for position, raw_count in enumerate(samples):
        # Estimate per-sample wall-clock time by spacing samples at 250 Hz and
        # treating the newest sample in the packet as arriving at callback time.
        timestamp = packet_receive_time - (
            (sample_count - 1 - position) / SAMPLE_RATE_HZ
        )
        sample_index = stats.total_samples
        stats.total_samples += 1

        record = SampleRecord(
            timestamp_unix_s=timestamp,
            sample_index=sample_index,
            raw_adc_count=raw_count,
            packet_sequence=sequence,
        )

        try:
            sample_queue.put_nowait(record)
        except asyncio.QueueFull:
            stats.host_queue_drops += 1
            now = time.monotonic()
            if now - stats.last_queue_warning_monotonic >= 1.0:
                stats.last_queue_warning_monotonic = now
                print(
                    "WARNING: host processing queue is full; decoded samples "
                    "are being dropped. Close other heavy programs or use "
                    "--no-plot.",
                    file=sys.stderr,
                )


def decode_status(data: bytes) -> str:
    if len(data) != 20:
        return f"status characteristic returned {len(data)} bytes, expected 20"

    (
        version,
        flags,
        sample_rate,
        dropped_samples,
        invalid_frames,
        total_valid_samples,
        ring_count,
        next_packet_sequence,
    ) = struct.unpack("<BBHIIIHH", data)

    return (
        f"version={version}, sample_rate={sample_rate} Hz, "
        f"ble_ready={bool(flags & 0x01)}, connected={bool(flags & 0x02)}, "
        f"subscribed={bool(flags & 0x04)}, "
        f"square_wave={bool(flags & 0x08)}, "
        f"device_dropped_samples={dropped_samples}, "
        f"invalid_ads_frames={invalid_frames}, "
        f"device_valid_samples={total_valid_samples}, ring_count={ring_count}, "
        f"next_packet_sequence={next_packet_sequence}"
    )


async def wait_or_stop(stop_event: asyncio.Event, seconds: float) -> None:
    if seconds <= 0:
        await asyncio.sleep(0)
        return

    try:
        await asyncio.wait_for(stop_event.wait(), timeout=seconds)
    except asyncio.TimeoutError:
        pass


async def connection_manager(
    args: argparse.Namespace,
    sample_queue: asyncio.Queue[SampleRecord],
    stats: ReceiverStats,
    stop_event: asyncio.Event,
) -> None:
    loop = asyncio.get_running_loop()

    while not stop_event.is_set():
        client: Optional[BleakClient] = None
        notifications_started = False
        unexpected_disconnect = False

        try:
            print(
                f"Scanning for BLE device '{DEVICE_NAME}' "
                f"for up to {args.scan_timeout:g} seconds..."
            )
            device = await BleakScanner.find_device_by_filter(
                matches_target_device,
                timeout=args.scan_timeout,
            )

            if device is None:
                print(
                    f"ERROR: '{DEVICE_NAME}' was not found. Confirm the XIAO "
                    "is powered, advertising, and not connected to another app.",
                    file=sys.stderr,
                )
                await wait_or_stop(stop_event, args.reconnect_delay)
                continue

            print(
                f"Found {DEVICE_NAME} at {getattr(device, 'address', 'unknown address')}."
            )

            disconnected_event = asyncio.Event()

            def on_disconnected(_client: BleakClient) -> None:
                loop.call_soon_threadsafe(disconnected_event.set)

            client = BleakClient(
                device,
                disconnected_callback=on_disconnected,
                timeout=args.connect_timeout,
            )
            await client.connect()

            if not client.is_connected:
                raise BleakError("connection attempt completed but client is not connected")

            sample_characteristic = client.services.get_characteristic(
                SAMPLE_CHARACTERISTIC_UUID
            )
            if sample_characteristic is None:
                raise BleakError(
                    "connected device does not expose the expected sample "
                    f"characteristic {SAMPLE_CHARACTERISTIC_UUID}"
                )

            print("Connected. Expected sample characteristic was found.")
            print(f"Negotiated ATT MTU: {client.mtu_size} bytes")

            try:
                status_data = bytes(
                    await client.read_gatt_char(STATUS_CHARACTERISTIC_UUID)
                )
                print(f"Device status: {decode_status(status_data)}")
            except BleakError as exc:
                print(
                    f"Status characteristic could not be read: {exc}",
                    file=sys.stderr,
                )

            generation = stats.begin_connection()

            def notification_callback(_sender: object, data: bytearray) -> None:
                # Bleak already schedules notification callbacks onto its active
                # asyncio loop. Decode immediately instead of adding a second
                # call_soon_threadsafe hop for every packet.
                decode_and_enqueue_packet(
                    bytes(data),
                    generation,
                    sample_queue,
                    stats,
                )

            await client.start_notify(
                sample_characteristic,
                notification_callback,
            )
            notifications_started = True
            print(
                "Subscribed to Channel 4 notifications. "
                "For diagnosis, run once with --no-plot: if packet loss "
                "remains, the bottleneck is the BLE link/firmware rather "
                "than Matplotlib."
            )

            stop_wait = asyncio.create_task(stop_event.wait())
            disconnect_wait = asyncio.create_task(disconnected_event.wait())

            done, pending = await asyncio.wait(
                {stop_wait, disconnect_wait},
                return_when=asyncio.FIRST_COMPLETED,
            )

            unexpected_disconnect = disconnect_wait in done and not stop_event.is_set()

            for task in pending:
                task.cancel()
            await asyncio.gather(*pending, return_exceptions=True)

            if unexpected_disconnect:
                print("BLE connection was lost; reconnecting...", file=sys.stderr)

        except (BleakError, OSError) as exc:
            print(
                f"BLE error: {exc}\n"
                "Check that Bluetooth is enabled, the Windows Bluetooth adapter "
                "is working, and Nykin-EMG is not connected elsewhere.",
                file=sys.stderr,
            )
            unexpected_disconnect = True

        finally:
            stats.invalidate_connection()

            if client is not None:
                if notifications_started and client.is_connected:
                    try:
                        await client.stop_notify(SAMPLE_CHARACTERISTIC_UUID)
                    except (BleakError, OSError) as exc:
                        print(
                            f"Could not stop notifications cleanly: {exc}",
                            file=sys.stderr,
                        )

                if client.is_connected:
                    try:
                        await client.disconnect()
                    except (BleakError, OSError) as exc:
                        print(
                            f"Could not disconnect cleanly: {exc}",
                            file=sys.stderr,
                        )

        if not stop_event.is_set() and unexpected_disconnect:
            await wait_or_stop(stop_event, args.reconnect_delay)


async def record_consumer(
    args: argparse.Namespace,
    sample_queue: asyncio.Queue[SampleRecord],
    stats: ReceiverStats,
    stop_event: asyncio.Event,
) -> None:
    csv_file = None
    csv_writer = None
    plotter: Optional[PlotProcessController] = None
    signal_filter = EmgFilter()
    button_csv_records: list[SampleRecord] = []
    settled_sample_index: Optional[int] = None
    save_tasks: set[asyncio.Task[None]] = set()

    try:
        if args.csv is not None:
            args.csv.parent.mkdir(parents=True, exist_ok=True)
            csv_file = args.csv.open("w", newline="", encoding="utf-8")
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(
                [
                    "timestamp_unix_s",
                    "sample_index",
                    "raw_adc_count",
                    "filtered_adc_count",
                    "detector_envelope",
                    "activity_detected",
                    "packet_sequence",
                ]
            )
            csv_file.flush()
            print(f"Recording CSV to: {args.csv.resolve()}")

        if not args.no_plot:
            plotter = PlotProcessController(args.seconds)
            print(
                "Plot window started in a separate process; Matplotlib can no "
                "longer block BLE notification handling."
            )

        last_plot_publish = time.monotonic()
        last_stats_print = time.monotonic()
        last_csv_flush = time.monotonic()

        while not stop_event.is_set() or not sample_queue.empty():
            batch: list[SampleRecord] = []

            try:
                first_record = await asyncio.wait_for(
                    sample_queue.get(),
                    timeout=SAMPLE_CONSUMER_WAIT_S,
                )
                batch.append(first_record)
            except asyncio.TimeoutError:
                pass

            while len(batch) < 2048:
                try:
                    batch.append(sample_queue.get_nowait())
                except asyncio.QueueEmpty:
                    break

            if batch:
                batch = [signal_filter.process_record(record) for record in batch]

                if settled_sample_index is None:
                    settled_sample_index = (
                        batch[0].sample_index
                        + int(round(CSV_SETTLE_TIME_S * SAMPLE_RATE_HZ))
                    )

                button_csv_records.extend(
                    record
                    for record in batch
                    if record.sample_index >= settled_sample_index
                )

                if args.print_samples:
                    for record in batch:
                        print(
                            f"{record.sample_index},{record.raw_adc_count},"
                            f"{record.filtered_adc_count:.6f},"
                            f"{record.detector_envelope:.6f},"
                            f"{int(record.activity_detected)}"
                        )

                if csv_writer is not None:
                    csv_writer.writerows(
                        (
                            f"{record.timestamp_unix_s:.6f}",
                            record.sample_index,
                            record.raw_adc_count,
                            f"{record.filtered_adc_count:.6f}",
                            f"{record.detector_envelope:.6f}",
                            int(record.activity_detected),
                            record.packet_sequence,
                        )
                        for record in batch
                    )

                if plotter is not None:
                    plotter.add_records(batch)

            now = time.monotonic()

            if (
                plotter is not None
                and now - last_plot_publish >= PLOT_SNAPSHOT_INTERVAL_S
            ):
                last_plot_publish = now
                plotter.publish_latest()

            if plotter is not None:
                for command, payload in plotter.poll_commands():
                    if command == "closed":
                        stop_event.set()
                    elif command == "save" and payload is not None:
                        records_snapshot = button_csv_records.copy()
                        task = asyncio.create_task(
                            asyncio.to_thread(
                                write_records_to_csv,
                                records_snapshot,
                                Path(payload),
                            )
                        )
                        save_tasks.add(task)
                        task.add_done_callback(save_tasks.discard)

                if not plotter.is_alive() and not stop_event.is_set():
                    print(
                        "Plot process exited unexpectedly; stopping receiver.",
                        file=sys.stderr,
                    )
                    stop_event.set()

            if csv_file is not None and now - last_csv_flush >= 1.0:
                last_csv_flush = now
                csv_file.flush()

            if now - last_stats_print >= STATS_PRINT_INTERVAL_S:
                last_stats_print = now
                plot_replacements = (
                    plotter.snapshot_replacements if plotter is not None else 0
                )
                total_expected_packets = stats.total_packets + stats.missing_packets
                packet_loss_percent = (
                    100.0 * stats.missing_packets / total_expected_packets
                    if total_expected_packets
                    else 0.0
                )
                print(
                    "Host totals: "
                    f"packets={stats.total_packets}, "
                    f"samples={stats.total_samples}, "
                    f"missing_packets={stats.missing_packets}, "
                    f"packet_loss={packet_loss_percent:.1f}%, "
                    f"malformed={stats.malformed_packets}, "
                    f"protocol_errors={stats.protocol_errors}, "
                    f"out_of_order={stats.out_of_order_packets}, "
                    f"host_queue_drops={stats.host_queue_drops}, "
                    f"queue_depth={sample_queue.qsize()}, "
                    f"plot_snapshot_replacements={plot_replacements}"
                )

    finally:
        if csv_file is not None:
            csv_file.flush()
            csv_file.close()

        if plotter is not None:
            plotter.close()

        if save_tasks:
            await asyncio.gather(*save_tasks, return_exceptions=True)


async def async_main(args: argparse.Namespace) -> ReceiverStats:
    stop_event = asyncio.Event()
    sample_queue: asyncio.Queue[SampleRecord] = asyncio.Queue(
        maxsize=HOST_QUEUE_MAX_RECORDS
    )
    stats = ReceiverStats()
    loop = asyncio.get_running_loop()
    stop_requested = False

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal stop_requested
        if not stop_requested:
            stop_requested = True
            print("\nStopping...")
        loop.call_soon_threadsafe(stop_event.set)

    signal.signal(signal.SIGINT, request_stop)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, request_stop)

    connection_task = asyncio.create_task(
        connection_manager(args, sample_queue, stats, stop_event)
    )
    consumer_task = asyncio.create_task(
        record_consumer(args, sample_queue, stats, stop_event)
    )

    try:
        await asyncio.gather(connection_task, consumer_task)
    finally:
        stop_event.set()

        for task in (connection_task, consumer_task):
            if not task.done():
                task.cancel()

        await asyncio.gather(
            connection_task,
            consumer_task,
            return_exceptions=True,
        )

    return stats


def main() -> int:
    args = parse_args()

    try:
        stats = asyncio.run(async_main(args))
    except KeyboardInterrupt:
        # Fallback for environments where the custom SIGINT handler is replaced.
        print("\nInterrupted.")
        return 130
    except (BleakError, OSError) as exc:
        print(f"Fatal Bluetooth error: {exc}", file=sys.stderr)
        return 1
    except Exception as exc:  # Keep unexpected failures understandable to users.
        print(f"Fatal error: {exc}", file=sys.stderr)
        return 1

    print(
        "Final totals: "
        f"packets={stats.total_packets}, samples={stats.total_samples}, "
        f"missing_packets={stats.missing_packets}, "
        f"malformed={stats.malformed_packets}, "
        f"protocol_errors={stats.protocol_errors}, "
        f"out_of_order={stats.out_of_order_packets}, "
        f"host_queue_drops={stats.host_queue_drops}"
    )
    return 0


if __name__ == "__main__":
    mp.freeze_support()
    raise SystemExit(main())