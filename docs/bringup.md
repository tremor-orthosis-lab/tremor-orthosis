# Sensor kit: wiring and first power-up (task J-1.2)

Goal: ESP32 + 2 × MPU-6050 on a breadboard, logging both IMUs at 500 Hz with no lost sample. Takes about one hour.

## 0. Before the hardware arrives (once)

- VS Code + the **PlatformIO IDE** extension.
- `python3 -m pip install pyserial` (for `tools/record_log.py`).
- MATLAB can open the repository folder (System Settings → Privacy & Security → Files and Folders → MATLAB → Documents).

## 1. Wiring

Both GY-521 boards go on the **same I²C bus**; the AD0 pin gives each one its own address.

| GY-521 pin | IMU1 — forearm | IMU2 — hand | ESP32 pin |
| --- | --- | --- | --- |
| VCC | ✔ | ✔ | **VIN** (5 V from USB; the GY-521 has its own 3.3 V regulator) |
| GND | ✔ | ✔ | **GND** |
| SCL | ✔ | ✔ | **GPIO 22** |
| SDA | ✔ | ✔ | **GPIO 21** |
| AD0 | → **GND** (address 0x68) | → **3V3** (address 0x69) | |
| XDA, XCL, INT | not connected | not connected | |

In practice: run four breadboard rails from the ESP32 (**VIN**, **GND**, **GPIO 21**, **GPIO 22**) and connect both boards to them, pin for pin. Then the only difference between the two boards is the AD0 wire.

Wires to the hand: 30–50 cm Dupont jumpers, loose enough not to pull when the wrist moves. If the I²C reads fail with long wires, lower `I2C_FREQ_HZ` to 100000 in `include/contracts.h`.

USB-C board: if it does not power up with a USB-C ↔ USB-C cable, use a **USB-A ↔ USB-C** cable (some boards lack the USB-C resistors).

## 2. Mounting on the arm (signs, contract C2)

- IMU1 on the **back of the forearm**, 5 cm before the wrist. IMU2 on the **back of the hand**.
- Both boards **components facing out** (away from the skin) and the **X arrow printed on the board pointing towards the fingers**.
- Then: x points to the fingers, z points out of the back of the hand, and **extension (back of the hand up) is positive**.
- Elastic strap or velcro, tight enough that the board does not wobble on the skin.

## 3. First power-up

1. **Board alone, no IMU wired.** Open the repository in VS Code, then:
   ```bash
   pio run -e mock -t upload
   pio device monitor
   ```
   Expected: `# tremor-orthosis: keep the hand still...`, then `# FAULT: IMU init failed` (normal, no IMU yet) and `# ready`.
   If the upload stays on `Connecting....`: hold the **BOOT** button until it starts writing.
2. **Unplug, wire the two IMUs, plug back.** In the monitor type `SCAN`:
   ```
   #   0x68 IMU1 forearm (AD0 -> GND)
   #   0x69 IMU2 hand (AD0 -> 3.3 V)
   ```
3. Press the **EN** (reset) button with the arm **still** (2 s gyro calibration) until `# ready`.
4. Type `WATCH on`. Five lines per second:
   ```
   theta  +12.3 deg | omega   +45.0 deg/s | theta_v  +12.1 theta_t  +0.2 deg | f 5.0 Hz | acc1 ... | acc2 ...
   ```
   Check, forearm horizontal, palm down:

   | Do | Expected |
   | --- | --- |
   | hand still, aligned with the forearm | `theta` ≈ 0 (± 5°), `acc1` and `acc2` z ≈ **+9.8** |
   | bend the hand **up** (extension) ~30° | `theta` ≈ **+30**, `omega` **positive** while moving |
   | bend the hand **down** (flexion) | `theta` and `omega` **negative** |
   | shake the hand at ~5 Hz | `f` goes to ≈ 5 Hz, `theta_t` oscillates |

   **If a sign is wrong, do not change the code**: a board is mounted upside down or rotated. Check the X arrows and that the components face out. If it still disagrees, note what you see and agree with your teammate before touching `contracts.h` (C2).
5. `WATCH off`, then `STATUS`: `loop_max` and `jitter_max` since the previous STATUS.

## 4. J-1.2 acceptance test (10 min recording)

Close the PlatformIO monitor (only one program can use the port), then:

```bash
python3 tools/record_log.py timing -s 600
```

Done when the script prints **0 lost samples, 500 Hz, worst loop < 1000 µs**, and `STATUS` shows `jitter_max` < 50 µs.

## If something goes wrong

| Symptom | Likely cause |
| --- | --- |
| No `/dev/cu.usbserial-*` (`ls /dev/cu.*`) | charge-only cable; or install the Silicon Labs CP210x driver |
| `SCAN` finds nothing | SDA/SCL swapped, VCC/GND swapped, loose breadboard contact |
| Only 0x68 | IMU2 AD0 not connected to 3V3 |
| `FAULT: IMU read failed 3x` while moving | loose wire to the hand; lower `I2C_FREQ_HZ` |
| Lost samples in the log | other program on the port, long/poor USB cable; the log uses ~80 % of the 921600 baud link |
| `theta` drifts slowly at rest | normal for the accelerometer angle; `theta_v` must stay steady |
