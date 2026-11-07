# Known Issues & Risks

Found while reading and documenting every file. **Nothing here was changed in the
code** (the clean-up was comments-only) — these are findings for you to decide on.
Each item says where to look and what a fix would be.

Legend — **S** security · **R** robot safety · **F** functional / documentation
mismatch · **H** repository hygiene. Severity: 🔴 act soon · 🟠 should fix · 🟡 minor.

Status date: 2026-11-07. Items were verified by reading code and checking this Pi;
none were reproduced on hardware.

---

## 🔴 Security

### S1 · A Google (Gemini) API key is committed and pushed
* **Where:** `multiprocess_code/all_4_legs/cur_test/BYTE_VISION/integrated.py`,
  the `genai.configure(api_key="…")` line near the top. First committed in
  `fb8dbdee` ("Add leg commander and swing balance scripts…"); that commit is on
  `origin/motor_codes` → `github.com/AmshuBelbase/Byte_01`. Whether the repository is
  public was not checked.
* **Impact:** anyone who can read the repository can use the key (billing / quota abuse).
* **Fix, in this order:**
  1. **Revoke / rotate the key** in the Google AI Studio / Cloud console. This is the
     only step that actually closes the hole — deleting the line does not, because it
     stays in git history.
  2. Read the new key from the environment: `genai.configure(api_key=os.environ["GEMINI_API_KEY"])`.
  3. Optional, after revoking: purge it from history with `git filter-repo` (needs a
     coordinated force-push). Not necessary once the key is dead.
* The key was deliberately **not** copied into any documentation.

### S2 · Unauthenticated network control, and `pickle` over the socket
* **Where:** `robot/main_with_ik.py` → `socket_listener_process` (`pickle.loads` on
  bytes from TCP :50000); `robot/state_manager_server_v2.py` (HTTP on `0.0.0.0:8000`,
  no authentication).
* **Impact:** `pickle.loads` on network input executes arbitrary code in the sender's
  chosen way — anyone who can reach port 50000 gets a shell as the user running the
  robot. Anyone who can reach :8000 can make the robot walk.
* **Mitigation now:** run only on a trusted/isolated network; firewall both ports
  (`sudo ufw allow from <your-ip> to any port 50000,8000`).
* **Fix:** replace pickle with JSON (`json.dumps` the same dict — every consumer is
  ours, and the payload is plain lists and floats) and bind to `127.0.0.1` unless
  remote control is needed; add a shared-secret header to the HTTP API.

---

## 🟠 Robot safety

### R1 · Every exit path makes the robot go limp
`finally:` in `main()` calls `disable_all()` (zero-current brake). Ctrl-C, a latched
fault, a watchdog trip, or an exception all collapse a standing robot. This is the
intended fail-safe, but it means: **support or sit the robot before stopping the
program.** A "sit down first, then disable" shutdown would be a safer default for
operator-initiated stops.

### R2 · One bad packet shuts the whole runtime down
`convert_coords_to_motor_targets` (called inside the live loop) has no `try/except`.
An unreachable foot target makes the IK raise `ValueError`, which propagates out of
`run_post_homing_live_control`, is caught in `main()` as "Runtime error", and ends in
R1. Validation (`validate_leg_payload`) only checks shape, not reachability.
**Fix:** catch `ValueError` around the IK call, log it, and keep the previous target.

### R3 · `main_with_ik.py n` (skip homing) is easy to misuse
Without homing the runtime assumes the motors still hold the zero set at the last
sit-down and that the robot is physically sitting. After any power cycle that is false,
and the first command eases the legs toward a pose that no longer means "sit".
**Fix:** refuse `n` unless a quick sanity check (e.g. all joint angles within a few
degrees of 0) passes.

### R4 · The state server assumes the robot starts sitting
`current_positions` starts at zeros and is never read back from the robot. Restarting
the server while the robot stands desynchronises them (the first gait cycle starts from
the wrong baseline). **Fix:** send `STAND`/`SIT` once at start-up, or add a "get pose"
message to the protocol.

### R5 · The voice scripts speak the *old* coordinate protocol
`BYTE_VA/byte_sit.py` / `byte_stand.py` (run by `BYTE_VA/app.py`) send absolute
coordinates like `[-9.65, 20, 6]`. `robot/` interprets packet values as **offsets from
`SIT_COORDS`** → a wrong or unreachable pose (and then R2). They were written for the
earlier runtime (`main_with_current_temp.py`). **Fix:** make them send the `robot/`
SIT/STAND offsets, or call the HTTP API.

---

## 🟠 Functional / documentation mismatches

### F1 · `keyboard_controller.py` cannot control `state_manager_server_v2.py`
1. It posts to `http://SOCKET_HOST:SOCKET_PORT` — port **50000**, the raw motor socket.
   The HTTP API is on **8000**.
2. It sends `{"state": …}`; every server version requires
   `{"new_state", "sender_id", "cycles"}` → HTTP 422.
3. It has no keys for `CW`/`CCW`.
**Fix:** `STATE_URL = f"http://{SOCKET_HOST}:8000/change-state"` and body
`{"new_state": state, "sender_id": "keyboard", "cycles": 1}`.
(The commit message "problem of too many requests" is explained by F2: the server
answers 429 while busy.)

### F2 · HTTP responses don't reflect what happened
`handle_state_change` awaits the *entire* motion before replying, always replies
`"result": "received"` and discards the dict returned by `change_state`. So: `JUMP`
(registered, not implemented) returns 200 and does nothing; a 3-cycle walk blocks the
client for seconds; a client that retries gets 429 (`lock.locked()`; requests are
rejected, not queued). **Fix:** return `change_state`'s result; run gaits as a
background task and reply immediately.

### F3 · Stand-pose constants disagree
`state_manager_server_v2.py`: STAND/SIT `dx = 1.5`. `mpu_leveling.py`: `STAND_DX = 4.0`.
`state_manager_server_v3.py`: `dx = 0`. If MPU levelling is enabled the feet jump
fore-aft when the correction packet arrives. **Fix:** define `STAND_*` once in
`config/robot_config.py`.

### F4 · "+Y is left" comments are wrong
`lib/trot_gait.py` and some older gait files say +Y = left. `SIT_COORDS` (left legs
Y ≈ −9.09), `mpu_leveling.py` and `tests/ik_sender.py` all say **+Y = right**, and the
state names (`RIGHT` = axis y, direction +1) agree. Only the wording is stale.

### F5 · Small stale strings
* `test_client.py` banner lists states "sit, stand, walk, climb"; the real set is
  sit, stand, forward, backward, left, right, cw, ccw.
* `tests/ik_sender.py`: "make +14 to stand" — STAND is `dz = −14`.
* `tests/joint_sender.py` error text names `main_raw_tester.py` (now `joint_tester.py`).
* `BYTE_VA/ranscriber.py` is a typo for `transcriber.py` (imported by that name).

### F6 · Loggers overwrite instead of append
`current_logger_thread_entry` / `temp_logger_thread_entry` open the CSV with mode `"w"`;
their docstrings say "append". Every start erases the previous run's log.

### F7 · The gait module needs the IMU library
`lib/trot_gait.py` has `from mpu_leveling import MPULeveler` (never used). Importing
the gait code therefore requires `smbus2` and the current directory to be `robot/`.
**Fix:** delete the import.

### F8 · The runtime's network address is machine-specific
`SOCKET_HOST = "10.203.242.34"` is used as a **bind** address, but the Pi's Wi-Fi
address is assigned by DHCP and changes with the network. When checked (2026-11-07)
`wlan0` was `10.194.233.176`, so `main_with_ik.py` would fail to start with
"Cannot assign requested address". `cur_test` hard-codes yet another IP (`10.154.75.34`).
**Fix:** bind to `0.0.0.0` (or `127.0.0.1` when everything runs on the Pi) and keep the
*client* target in one setting that is easy to change per network.

### F9 · IK prints on every call
`RoboticLeg.inverse_kinematics` and `ik_to_motor_deg` print one line each per call:
8 lines per packet, ~800 lines/s while walking. Costs CPU on the Pi and buries real
messages. **Fix:** guard with a `DEBUG` flag.

### F10 · Dead or unfinished code
* `ControlTuning`: `seconds_per_deg`, `max_search_time_s`, `zero_feedback_timeout_s`,
  `stall_progress_epsilon_deg`, `stall_timeout_s` are never read.
* `FaultCode`: only `NO_FEEDBACK`, `SEND_FAIL`, `MOTOR_ERROR` are ever raised.
* `lib/trot_gait_lib.py` is imported by nothing; `flatten_leg_payload`, the `legs` list
  and `import numpy` in `lib/inverse_kinematics.py` are unused.
* `archive/04_imu_balancing/level_stand.py`: the IMU read is a `TODO` stub.
* `lib/trot_gait.py` `_gait_loop` is a print-only dry run (`# TODO: send payload`).

### F11 · The MPU-6050 did not answer on I²C (observed 2026-11-07)
A read-only probe of address `0x68` on bus 1 returned `OSError: [Errno 121] Remote I/O
error`, so the IMU is unplugged, unpowered, on another address/bus, or faulty. Effects:
`state_manager_server_v3.py` crashes at import (it creates an `MPULeveler` immediately),
`mpu_leveling.py` and the IMU experiments in `cur_test/` cannot run.
`state_manager_server_v2.py` is unaffected (its leveler is commented out).
Check: `i2cdetect -y 1` should show `68`; see `HARDWARE_SETUP.md` §4.

### F12 · `motor_config.json` has `flipped: false` for all 12 motors
Not a bug, but worth knowing: polarity is currently absorbed by the sign conventions in
the IK and in `SIT_TARGETS_DEG` (hip and thigh targets of the right legs are the
negatives of the left legs', to within a few degrees).
If a motor is ever re-mounted, that is where to look.

---

## 🟡 Repository hygiene

### H1 · The repository is very heavy
`.git` pack ≈ **3 GB**; the working tree is ≈ **5.9 GB**, of which the voice
assistant's virtualenv `BYTE_VA/byte_env/` is 5.7 GB (untracked, correctly ignored).
Tracked binaries include `yolov8l-seg.pt` (88 MB), the sherpa-onnx KWS model
(tarball 17 MB + extracted ONNX files), `yolov8n-seg.pt`, `silero_vad.onnx`.
`download_models.py` already fetches the voice models, so they needn't be in git.
**Fix:** `git rm --cached` the model files (the new `.gitignore` then keeps them out);
consider `git lfs` or a release asset for the YOLO weights; trimming *history* needs
`git filter-repo`.

### H2 · Build/log artefacts are tracked
`BYTE_VA/build/`, `install/`, `log/` (colcon markers and a 4 MB build log) — ROS
leftovers. `git rm -r --cached` them.

### H3 · Runtime CSV logs are tracked and change every run
`robot/logs/*.csv`, `cur_test/motor_*.csv`, `archive/04…/motor_*.csv`. They make every
`git status` noisy. `git rm --cached` them (the new `.gitignore` covers future ones).
Keep `cur_test/motor_urdf_stand.csv` and `kriss_exp_recording.csv` if they are reference data.

### H4 · Absolute paths
`cur_test/main_with_current_temp.py`, `main_with_ik.py`, `motor_current_plotter.py`,
`archive/04…/main_with_*.py` write to / read from
`/home/byte/ak60_motor_control/multiprocess_code/all_4_legs/cur_test/…`; the voice
virtualenv is not relocatable. This is why `cur_test/` could not be moved.
**Fix:** use paths relative to `__file__`; recreate the venv from
`BYTE_VA/requirements.txt` after moving.

### H5 · Duplicate code
The AK60 driver exists in 9 copies (4 distinct versions; 4 of them, including the one in
`robot/lib/`, were byte-identical before documentation), the bus runtime / homing / IK in 3 variants each,
`main_with_ik.py` in 3. Only `robot/` is the documented source of truth. Once
`cur_test/` is merged or retired, the copies can go.

### H6 · No tests, CI or license
There is no automated test for the maths (a ready-made check: `IK(SIT_COORDS)` must give
zero angles, and `FK(IK(p)) == p`) and no `LICENSE`. A `robot/requirements.txt`
now exists; adding a `tests/test_ik.py` using the two checks above would be a cheap,
valuable start.
