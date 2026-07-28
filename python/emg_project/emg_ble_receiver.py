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
HOST_QUEUE_MAX_RECORDS = 4096
PLOT_UPDATE_INTERVAL_S = 0.05
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

    def begin_connection(self) -> int:
        """Start a new sequence-tracking epoch and return its generation ID."""
        self.connection_generation += 1
        self.last_sequence = None
        return self.connection_generation

    def invalidate_connection(self) -> None:
        """Ignore notification work that was queued by an old connection."""
        self.connection_generation += 1
        self.last_sequence = None


class LivePlot:
    """Two synchronized Matplotlib plots updated from the asyncio/main thread."""

    def __init__(
        self,
        seconds: float,
        stop_event: asyncio.Event,
        save_csv_callback: object,
    ) -> None:
        # Lazy import allows --no-plot to run on headless systems.
        import matplotlib.pyplot as plt
        from matplotlib.widgets import Button

        self._plt = plt
        self._seconds = seconds
        self._max_points = max(1, int(round(seconds * SAMPLE_RATE_HZ)))
        self._sample_indices: deque[int] = deque(maxlen=self._max_points)
        self._raw_values: deque[int] = deque(maxlen=self._max_points)
        self._filtered_values: deque[float] = deque(maxlen=self._max_points)

        plt.ion()
        self._figure, axes = plt.subplots(2, 1, sharex=True)
        self._raw_axes = axes[0]
        self._filtered_axes = axes[1]
        self._figure.subplots_adjust(bottom=0.14, hspace=0.28)
        self._figure.suptitle("Nykin-EMG — ADS1299 Channel 4")

        (self._raw_line,) = self._raw_axes.plot([], [])
        self._raw_axes.set_title("Raw / Unfiltered")
        self._raw_axes.set_ylabel("Raw ADC count")
        self._raw_axes.grid(True)
        self._raw_axes.set_xlim(-self._seconds, 0.0)

        (self._filtered_line,) = self._filtered_axes.plot([], [])
        self._filtered_axes.set_title("Filtered")
        self._filtered_axes.set_xlabel("Time relative to newest sample (s)")
        self._filtered_axes.set_ylabel("Filtered ADC count")
        self._filtered_axes.grid(True)
        self._filtered_axes.set_xlim(-self._seconds, 0.0)

        button_axes = self._figure.add_axes([0.78, 0.025, 0.18, 0.055])
        self._save_button = Button(button_axes, "Save CSV")
        self._save_button.on_clicked(lambda _event: save_csv_callback())

        loop = asyncio.get_running_loop()

        def on_close(_event: object) -> None:
            loop.call_soon_threadsafe(stop_event.set)

        self._figure.canvas.mpl_connect("close_event", on_close)
        plt.show(block=False)

    def add_records(self, records: list[SampleRecord]) -> None:
        for record in records:
            # The same sample index is used for both signals, keeping the plots
            # aligned even when samples arrive in multi-sample BLE packets.
            self._sample_indices.append(record.sample_index)
            self._raw_values.append(record.raw_adc_count)
            self._filtered_values.append(record.filtered_adc_count)

    def refresh(self) -> None:
        if self._sample_indices:
            newest_index = self._sample_indices[-1]
            x_values = [
                (sample_index - newest_index) / SAMPLE_RATE_HZ
                for sample_index in self._sample_indices
            ]

            self._raw_line.set_data(x_values, list(self._raw_values))
            self._filtered_line.set_data(x_values, list(self._filtered_values))

            # Each graph gets its own y-scale because raw offsets and filtered
            # activity can have very different amplitudes. Their shared x-axis
            # keeps both displays synchronized in time.
            self._raw_axes.relim()
            self._raw_axes.autoscale_view(scalex=False, scaley=True)
            self._filtered_axes.relim()
            self._filtered_axes.autoscale_view(scalex=False, scaley=True)
            self._filtered_axes.set_xlim(-self._seconds, 0.0)

            self._figure.canvas.draw_idle()

        # Runs the GUI event loop briefly. This is intentionally not called
        # from the BLE notification callback.
        self._plt.pause(0.001)

    def close(self) -> None:
        if self._plt.fignum_exists(self._figure.number):
            self._plt.close(self._figure)


def save_records_with_dialog(records: list[SampleRecord]) -> None:
    if not records:
        print(
            f"No settled samples to save yet. Wait at least "
            f"{CSV_SETTLE_TIME_S:g} seconds after readings begin."
        )
        return

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

    if not filename:
        return

    path = Path(filename)
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
            print(
                f"WARNING: missing {forward_distance} BLE packet(s); "
                f"expected sequence {expected}, received {sequence}",
                file=sys.stderr,
            )
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
                # Bleak backends may invoke callbacks differently. Schedule all
                # decoding onto the asyncio thread, where asyncio.Queue is safe.
                loop.call_soon_threadsafe(
                    decode_and_enqueue_packet,
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
            print("Subscribed to Channel 4 notifications.")

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
    live_plot = None
    signal_filter = EmgFilter()
    button_csv_records: list[SampleRecord] = []
    settled_sample_index: Optional[int] = None

    def save_button_csv() -> None:
        save_records_with_dialog(button_csv_records.copy())

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
            live_plot = LivePlot(args.seconds, stop_event, save_button_csv)

        last_plot_update = time.monotonic()
        last_stats_print = time.monotonic()
        last_csv_flush = time.monotonic()

        while not stop_event.is_set() or not sample_queue.empty():
            batch: list[SampleRecord] = []

            try:
                first_record = await asyncio.wait_for(
                    sample_queue.get(),
                    timeout=PLOT_UPDATE_INTERVAL_S,
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

                if live_plot is not None:
                    live_plot.add_records(batch)

            now = time.monotonic()

            if live_plot is not None and now - last_plot_update >= PLOT_UPDATE_INTERVAL_S:
                last_plot_update = now
                live_plot.refresh()

            if csv_file is not None and now - last_csv_flush >= 1.0:
                last_csv_flush = now
                csv_file.flush()

            if now - last_stats_print >= STATS_PRINT_INTERVAL_S:
                last_stats_print = now
                print(
                    "Host totals: "
                    f"packets={stats.total_packets}, "
                    f"samples={stats.total_samples}, "
                    f"missing_packets={stats.missing_packets}, "
                    f"malformed={stats.malformed_packets}, "
                    f"protocol_errors={stats.protocol_errors}, "
                    f"out_of_order={stats.out_of_order_packets}, "
                    f"host_queue_drops={stats.host_queue_drops}"
                )

    finally:
        if csv_file is not None:
            csv_file.flush()
            csv_file.close()

        if live_plot is not None:
            live_plot.close()


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
    raise SystemExit(main())