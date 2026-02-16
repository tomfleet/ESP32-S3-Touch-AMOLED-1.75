# LiDAR UDP Display Interface (Current Firmware Contract)

Purpose: define the **actual UDP stream emitted today** by this firmware so a receiver/display can integrate without reverse-engineering.

## 1) Transport Contract

- Protocol: UDP/IPv4
- Receiver bind (typical): `0.0.0.0:8765` (port configurable in firmware)
- Sender target: configured in firmware (`APP_UDP_HOST`, `APP_UDP_PORT`)
- Delivery: best-effort (packet loss/out-of-order possible)
- Framing: **one complete JSON object per UDP datagram**
- Encoding: UTF-8 JSON text

Important:
- There is **no binary header**.
- There is **no magic/version/msg_type/packet_index/packet_count** in the UDP payload.
- Per-scan packet splitting/reassembly is **not used** in the current implementation.

## 2) UDP Payload Format (JSON)

Each datagram is a JSON object shaped like:

```json
{
  "source": "LD06_0",
  "scan": 1234,
  "speed": 3595,
  "speed_rpm": 599.17,
  "crc_fail": 0,
  "scan_t0_us": 1234567890,
  "scan_t1_us": 1234667890,
  "scan_period_us": 100000,
  "node_color": "green",
  "motor_pwm_duty_pct": 42.5,
  "motor_target_rpm": 600.0,
  "imu_accel_fs_g": 2.0,
  "imu_gyro_fs_dps": 245.0,
  "imu_sync": "interp",
  "imu": [roll_deg, pitch_deg, yaw_deg, ax_g, ay_g, az_g],
  "points": [[angle_deg, range_mm, intensity], ...],
  "imu_points": [[roll_deg, pitch_deg, yaw_deg], ...]
}
```

### 2.1 Field Definitions

| Field | Type | Meaning |
|---|---|---|
| `source` | string | Node/source tag from firmware task config |
| `scan` | integer | Monotonic scan counter (per source) |
| `speed` | integer | Raw LD06 speed value from parser |
| `speed_rpm` | number | Computed speed in RPM |
| `crc_fail` | integer | Cumulative LD06 CRC-fail counter |
| `scan_t0_us` | integer | Scan start timestamp (microseconds, firmware monotonic timebase) |
| `scan_t1_us` | integer | Scan end timestamp (microseconds, firmware monotonic timebase) |
| `scan_period_us` | integer | `scan_t1_us - scan_t0_us` |
| `node_color` | string | Current configured node color preset |
| `motor_pwm_duty_pct` | number | Active motor PWM duty percent (0 if PWM disabled) |
| `motor_target_rpm` | number | Target RPM used by motor controller |
| `imu_accel_fs_g` | number | IMU accel full-scale setting in g |
| `imu_gyro_fs_dps` | number | IMU gyro full-scale setting in deg/s |
| `imu_sync` | string | Current value: `"interp"` |
| `imu` | array[6] number | `[roll_deg, pitch_deg, yaw_deg, ax_g, ay_g, az_g]` |
| `points` | array of 3-number arrays | Each point: `[angle_deg, range_mm, intensity]` |
| `imu_points` | array of 3-number arrays | Per-point interpolated IMU: `[roll_deg, pitch_deg, yaw_deg]` |

### 2.2 Points Array Details

`points[i] = [angle_deg, range_mm, intensity]`

- `angle_deg`: float degrees (0..360 domain from parser)
- `range_mm`: integer millimeters
- `intensity`: integer raw intensity

Notes:
- UDP payload is decimated in firmware for size control.
- Current firmware caps UDP points per scan (currently 60 max in sender path).
- `imu_points` is generated using the same decimation step as `points`.

## 3) Receiver Behavior (Recommended)

## 3.1 Datagram Handling

- Read one UDP datagram.
- Decode UTF-8, parse JSON.
- If parse fails, drop packet and increment `packets_bad_json`.
- Validate required keys at minimum: `scan`, `points`.

## 3.2 Ordering / Gaps

Because UDP is best-effort:

- Track last accepted `scan` per `source`.
- If incoming `scan <= last_scan`, treat as duplicate/old and ignore.
- If incoming `scan > last_scan + 1`, count a scan gap.
- No packet reassembly is needed (single-datagram frame).

## 3.3 Minimal Implementation API

```text
on_udp_datagram(bytes) -> Optional[ScanFrame]

ScanFrame:
  source: str
  scan_id: int            # from "scan"
  speed_raw: int          # from "speed"
  speed_rpm: float
  crc_fail: int
  t0_us: int
  t1_us: int
  period_us: int
  points: List[Point]
  imu_snapshot: Optional[ImuSnapshot]
  imu_points: List[ImuRpy]

Point:
  angle_deg: float
  range_mm: float
  intensity: int

ImuSnapshot:
  roll_deg: float
  pitch_deg: float
  yaw_deg: float
  ax_g: float
  ay_g: float
  az_g: float
```

## 4) Circular Display Mapping Contract

## 4.1 Geometry

- Display center: `(cx, cy)` in pixels
- Display radius: `r_px`
- Range window: `[range_min_mm, range_max_mm]`

Radial normalization:

$$
r_n = clamp\left(\frac{range\_mm - range\_min\_mm}{range\_max\_mm - range\_min\_mm}, 0, 1\right)
$$

Pixel radius:

$$
r_p = r_n \cdot r_{px}
$$

Angle mapping (clockwise, 0° at top):

$$
\theta = (90 - angle\_deg) \cdot \pi/180
$$

Cartesian:

$$
x = cx + r_p \cdot cos(\theta),\quad y = cy - r_p \cdot sin(\theta)
$$

## 4.2 Styling

- Skip points with out-of-window range.
- Map `intensity` to brightness/color as desired.
- Optional persistence: fade previous frame each refresh.

## 5) Useful Runtime Counters

Suggested receiver counters:

- `packets_rx`
- `packets_bad_json`
- `packets_missing_required_fields`
- `packets_dup_or_old_scan`
- `scan_id_gaps`
- `frames_accepted`

## 6) Reference Validation Cases (Current JSON Protocol)

## 6.1 Normal ordered scans

- Receive `scan=100`, then `101`.
- Expect two accepted frames, no gaps.

## 6.2 Duplicate/late scan

- Receive `scan=200`, then `199` (or another `200`).
- Expect second packet ignored as old/duplicate.

## 6.3 Gap detection

- Receive `scan=300`, then `303`.
- Expect `scan_id_gaps += 2`.

## 6.4 JSON corruption

- Datagram is not valid JSON.
- Expect parse failure counter increment, no frame emit.

## 7) Integration Knobs

- `bind_ip`, `bind_port`
- `range_min_mm`, `range_max_mm`
- `render_fps`
- `max_points_display` (optional local downsample)
- `drop_old_or_duplicate_scans` (bool)

## 8) Compatibility Note

This document describes the **current firmware JSON UDP stream**.
If firmware later adds a binary protocol with header/magic/version, treat that as a new protocol version and keep this JSON path supported unless intentionally deprecated.
