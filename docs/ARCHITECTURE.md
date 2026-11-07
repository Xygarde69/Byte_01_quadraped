# Architecture

How BYTE-01's software is put together, from a keypress down to CAN frames.
Everything described here lives in [`robot/`](../robot/); file-by-file detail is
in [`CODE_MAP.md`](CODE_MAP.md).

## 1. The big picture

```mermaid
flowchart LR
    subgraph Operator["Operator side (any machine)"]
        KB["keyboard_controller.py<br/>test_client.py"]
    end

    subgraph Pi["Raspberry Pi"]
        SM["state_manager_server_v2.py<br/>FastAPI :8000<br/>state machine + gaits"]
        subgraph RT["main_with_ik.py  (motor runtime)"]
            SOCK["socket_listener_process<br/>(own OS process)"]
            LIVE["live control loop<br/>100 Hz, main thread"]
            B0["BusRuntime can0<br/>+ listener thread"]
            B1["BusRuntime can1<br/>+ listener thread"]
            LOG["current / temp<br/>CSV loggers"]
        end
        IMU["mpu_leveling.py<br/>(optional, I²C)"]
    end

    KB -- "HTTP POST /change-state" --> SM
    SM -- "TCP :50000<br/>pickled foot offsets" --> SOCK
    IMU -. "same TCP protocol" .-> SOCK
    SOCK -- "mp.Queue(1)<br/>newest packet only" --> LIVE
    LIVE --> B0 & B1
    B0 -- "can0  1 Mbit/s" --> M0["motors 1–6<br/>FL, BL"]
    B1 -- "can1  1 Mbit/s" --> M1["motors 7–12<br/>FR, BR"]
    B0 & B1 --> LOG
```

Three programs cooperate, each doing one job:

| Program | Job | Knows about | Does **not** know about |
|---|---|---|---|
| `state_manager_server_v2.py` | Turn *intents* ("FORWARD") into a stream of foot positions | gaits, timing, HTTP | motors, CAN, IK |
| `main_with_ik.py` | Turn foot positions into motor commands, safely | IK, gear ratios, limits, CAN | gaits, HTTP |
| `lib/*` | Reusable pieces: driver, bus runtime, homing, IK, gait maths | their own domain | each other's callers |

This separation is the main design idea: **you can replace the walking logic
without touching motor safety, and vice-versa.** The only contract between the
two programs is the packet in §3.

## 2. Software layers

```
┌───────────────────────────────────────────────────────────────────────────┐
│ clients      keyboard_controller · test_client · (web UI / gamepad / voice)│  intent
├───────────────────────────────────────────────────────────────────────────┤
│ behaviour    state_manager_server_v2  ── lib/trot_gait.py                 │  gaits
├──────────────────────────  TCP :50000, pickled dict  ─────────────────────┤
│ runtime      main_with_ik.py   (live loop, rate limiter, shutdown)        │  safety
├───────────────────────────────────────────────────────────────────────────┤
│ kinematics   lib/inverse_kinematics.py  (+ config/robot_config.py)        │  maths
│ homing       lib/homing_controller.py                                     │
│ bus runtime  lib/can_runtime.py  (state, listener, watchdogs, faults)     │
├───────────────────────────────────────────────────────────────────────────┤
│ driver       lib/ak60_v3_control.py   (MIT frames, status decode)         │  wire
├───────────────────────────────────────────────────────────────────────────┤
│ OS / HW      SocketCAN can0 / can1 · MCP2515 · 12 × AK60-6 V3.0           │
└───────────────────────────────────────────────────────────────────────────┘
```

Rule of thumb: **a layer only calls the one directly below it.** `main_with_ik`
never builds a CAN frame; the driver never knows what a leg is.

## 3. The packet protocol (the contract between behaviour and runtime)

One TCP connection per packet — connect, send, close. Payload is
`pickle.dumps` of:

```python
{
  "fl": [dx, dy, dz],   # front-left  foot offset, cm, from the SITTING pose
  "bl": [dx, dy, dz],   # back-left
  "fr": [dx, dy, dz],   # front-right
  "br": [dx, dy, dz],   # back-right
  "speed": 400.0,       # OPTIONAL joint speed limit, deg/s (default 120)
}
```

* All four legs are mandatory; the runtime drops malformed packets and keeps going.
* Values are **absolute offsets from `SIT_COORDS`**, not increments: `0,0,0` is
  "sit", `dz = −14` is "stand". The state manager keeps the running pose and
  sends it each tick.
* Only the **newest** packet matters. The hand-off queue has length 1 and stale
  packets are discarded, so a slow runtime can never build a backlog.
* `speed` persists until the next packet changes it. Gaits send 500 deg/s;
  static poses use the default 120 deg/s, which produces the slow, gentle
  sit ⇄ stand transitions.

> ⚠ The payload is a *pickle*. See [KNOWN_ISSUES §S2](KNOWN_ISSUES.md).

Other message shapes that exist: `tests/joint_sender.py` sends
`{motor_id: angle_deg, "speed": …}` to `tests/joint_tester.py` (a separate
raw-joint runtime for bench work); it is not understood by `main_with_ik.py`.

## 4. From foot position to motor torque

What happens every 10 ms inside `run_post_homing_live_control`:

```mermaid
flowchart TD
    A["newest packet<br/>(dx,dy,dz) per leg"] --> B["+ SIT_COORDS[leg]<br/>→ absolute foot position (cm)"]
    B --> C["inverse kinematics<br/>(θ1,θ2,θ3) in degrees"]
    C --> D["− IK angles of the sit pose<br/>→ angle relative to sitting"]
    D --> E{"flipped?"}
    E -- yes --> E2["negate"]
    E -- no --> F
    E2 --> F["× gear_ratio<br/>(knees 1.5)"]
    F --> G["rate limiter<br/>≤ speed × gear_ratio deg/s per tick"]
    G --> H["MIT frame per motor<br/>kp, kd from motor_config.json, velocity 0, torque 0"]
    H --> I["CAN"]
```

**Why "relative to the sit pose"?** At start-up every motor is driven to the
sitting angles (`SIT_TARGETS_DEG`) and then *re-zeroed there*. After that, motor
angle 0 = sitting, so the IK output must be expressed relative to the IK angles
of `SIT_COORDS`. The two tables in `config/robot_config.py` therefore describe
the **same physical pose** in two ways and must be kept consistent.

**What the motor does with the frame** (MIT mode, evaluated inside the motor):

```
torque = kp · (target_angle − angle) + kd · (0 − speed) + 0
```

i.e. a spring-damper pulling toward the target. `kp ≈ 300–400`, `kd ≈ 1.5`
(`config/motor_config.json`). The rate limiter is what keeps the spring target
moving smoothly; the spring itself is only as stiff as `kp`.

### Coordinate frames

```
              +Z (up)
               │      +X (forward)
               │    ╱
               │  ╱
               │╱______ +Y (right)
```

Origin at each leg's hip. Left legs have negative Y at rest (`SIT_COORDS`), which
is how `+Y = right` was determined (older comments in a few gait files still say
"+Y left"; see KNOWN_ISSUES F4).

### Leg model

Three revolute joints; link lengths in cm: `L1 = 5.995` (fore-aft hip offset),
`Lc = 9.094` (lateral offset of the thigh plane), `L2 = 22.0` (thigh),
`L3 = 21.5` (shank). Forward kinematics:

```
Le = L2·cosθ2 + L3·cos(θ2+θ3)         X = −L1 − Le
Lp = L2·sinθ2 + L3·sin(θ2+θ3)         Y =  Lc·cosθ1 + Lp·sinθ1
                                      Z = −Lc·sinθ1 + Lp·cosθ1
```

The IK inverts this analytically (closed form, no iteration): θ1 from the
tangent to the `Lc` cylinder, θ2/θ3 from the planar two-link solution. It was
checked numerically: `FK(IK(p)) − p` is below 1e-13 cm on sit/stand-range
targets, and `IK(SIT_COORDS)` minus the sit reference is exactly 0 for all legs.

## 5. Threads, processes and who owns what

| Name | Kind | Rate | Responsibility |
|---|---|---|---|
| main thread | thread | 100 Hz | homing coordination → sit/zero → live loop → shutdown |
| `socket_listener_process` | **process** | event | accept TCP, un-pickle, validate, put on queue |
| `controller_thread_entry` ×2 | threads | 100 Hz | homing, one per CAN bus, only while homing |
| python-can `Notifier` ×2 | threads | event | decode status frames into `MotorState` |
| `CurrentLogger`, `TempLogger` | threads | 5 Hz | write `robot/logs/*.csv` |
| state server | asyncio loop | 100 Hz while gaiting | HTTP + gait ticks |

The socket reader is a separate **process** so that network I/O and un-pickling
can never stall the 100 Hz motor loop (Python's GIL would otherwise couple them).

Shared state is confined to `BusRuntime.states` behind one re-entrant lock;
readers get **deep copies**, so no consumer ever sees a half-updated motor.

## 6. Start-up sequence

```mermaid
sequenceDiagram
    participant Main as main thread
    participant T0 as thread can0
    participant T1 as thread can1
    participant M as motors

    Main->>M: wait for status frames from all 12 (≤ 5 s)
    Main->>M: 1 s soft hold at boot pose
    opt argument "y" (homing)
        par Phase 1
            T0->>M: BL leg: thigh → knee → hip into hard stops
        and
            T1->>M: FR leg: thigh → knee → hip into hard stops
        end
        T0-->>T1: barrier (each holds until both finish)
        par Phase 2
            T0->>M: FL leg
        and
            T1->>M: BR leg
        end
        Main->>T0: takeover (single commander from here)
    end
    Main->>M: hold 1 s → slew to SIT_TARGETS_DEG → set temporary origin (0° = sit)
    Main->>Main: start CSV loggers + socket process
    loop every 10 ms
        Main->>M: newest packet → IK → rate limit → MIT frames
    end
```

Homing detail (what "drive into a hard stop" means) is in the module docstring of
`robot/lib/homing_controller.py`: profiled approach → slow creep → current
spike *latches* contact → confirmed by sustained current or stall → temporary
zero → back off to a known pose.

## 7. Safety model

| Layer | Mechanism | Effect |
|---|---|---|
| Driver | all MIT inputs clamped to the motor's ranges | no out-of-range frames |
| Runtime | slew-rate limiter on every motor | no joint can be commanded faster than `speed × gear` deg/s |
| Runtime | `validate_leg_payload` | malformed packets dropped, loop continues |
| Bus | per-motor stale-feedback watchdog (0.15 s) | latched fault → all motors disabled |
| Bus | whole-bus silence watchdog (0.40 s), BUS-OFF, listener death | latched fault |
| Motor | any non-zero error byte (over-temp, over-current, …) | latched fault |
| Homing | `max_search_travel_deg` (220°), contact latch + confirmation | aborts runaway search |
| Shutdown | `finally:` always runs `disable_all()` | motors go limp |

Faults are **latched and fatal by design**: the response to *anything*
unexpected is "stop commanding, go limp". The corollary is that the robot
collapses when that happens — see KNOWN_ISSUES R1/R2.

## 8. Extending it

* **New behaviour (a gait, a gamepad, a web UI):** write a client that sends the §3
  packet. You never need to touch `main_with_ik.py`. The stateless engine in
  `lib/trot_gait_lib.py` is a good template — it computes targets from absolute
  positions and cannot drift if a tick is dropped.
* **New static pose:** add an entry to `static_states` in the state manager.
* **Tuning a joint:** edit `config/motor_config.json` (kp/kd/flip/gear) — no code.
* **Different sit pose:** change `SIT_COORDS` **and** `SIT_TARGETS_DEG` together.
* **Different network:** edit `SOCKET_HOST` in `config/robot_config.py`.
