"""
state_manager_server_v2.py — HTTP "state machine" front end for BYTE-01  (CURRENT)
══════════════════════════════════════════════════════════════════════════════

Run (from ``robot/``) on the Pi, AFTER ``main_with_ik.py`` is up and showing
"Socket receiver process started":

    python3 state_manager_server_v2.py        # serves http://0.0.0.0:8000

WHERE IT SITS
    keyboard_controller.py / test_client.py / a web UI ──HTTP :8000──▶ THIS
    THIS ──pickled foot offsets over TCP :50000──▶ main_with_ik.py ──▶ motors

    It translates high-level intents ("STAND", "FORWARD" …) into the stream of
    foot-position packets that ``main_with_ik.py`` understands.  It never talks
    to the motors, CAN or IK itself.

HTTP API
    POST /change-state      Content-Type: application/json
         {"new_state": "FORWARD",   # case-insensitive, see below
          "sender_id": "my_client", # free text, only used in logs/errors
          "cycles":    3}           # gait cycles to run (ignored by SIT/STAND)

    Responses
        200  {"sender_id", "requested_state", "cycles", "result": "received"}
        400  unknown state
        422  body doesn't match the model (all three fields are required)
        429  robot busy — another request is still executing (no queueing)
        503  could not reach the motor runtime on :50000
    ⚠ The 200 is only returned AFTER the motion has finished (the handler
      awaits it), so a 3-cycle walk takes seconds to answer.  Set client
      timeouts accordingly.  See also the notes at ``handle_state_change``.

STATES
    Static  (move once, then wait 2 s):
        SIT     all feet to the sit pose           offset (1.5, 0, 0)
        STAND   all feet 14 cm lower               offset (1.5, 0, −14)
    Dynamic (run ``cycles`` gait cycles, then return to STAND):
        FORWARD / BACKWARD    trot along X          (axis 'x', direction +1 / −1)
        RIGHT   / LEFT        sideways trot         (axis 'y', direction +1 / −1)
        CW      / CCW         turn in place         (rotation gait)
        JUMP                  registered but NOT implemented — the request is
                              accepted and nothing moves.
    Joint nudge (relative to the CURRENT pose, repeatable):
        UP      every leg: knee opens by UP_KNEE_DEG (5°) and the thigh turns
                UP_THIGH_DEG (2.5°) the opposite way, which straightens the
                leg under the hip and raises the body ~1.3 cm per press.
                Done as IK → change joint angles → FK, so it is sent as a
                normal foot-position packet.  400 if the leg would get too
                straight (|knee| < UP_MIN_KNEE_DEG) or IK fails.  The next
                STAND/gait resets the height to the normal STAND pose.

COORDINATES SENT TO THE ROBOT
    Each packet carries, per leg, an **offset in cm from SIT_COORDS** (the
    absolute pose of the foot relative to the sitting pose; not an increment).
    ``self.current_positions`` is that running pose.  The gait controllers
    return per-tick increments, which are added into it.

CONCURRENCY
    One ``asyncio.Lock`` serialises motions.  If a motion is in progress a new
    request is *rejected* with 429 (not queued) — which is why a client that
    spams requests sees "too many requests" (see the commit "problem of too
    many requests").

VERSIONS in this folder
    state_manager_server.py     v1: no rotation, SIT dx = 0
    state_manager_server_v2.py  v2 (this file): + CW/CCW rotation, SIT/STAND dx = 1.5
    state_manager_server_v3.py  experiment: MPU leveling enabled, no rotation
    MPU leveling is commented out here (``# self.leveler``) because
    ``MPULeveler`` talks to the IMU over I²C.
"""
import pickle
import socket
import asyncio
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
import math
from config.robot_config import SOCKET_HOST, SOCKET_PORT, LEG_ORDER, MAX_LIVE_DEG_PER_S, SIT_COORDS
from lib.inverse_kinematics import leg_solver
from lib.trot_gait import TrotGaitController, RotationGaitController
import time
import copy
# from mpu_leveling import MPULeveler


# UP state tuning (joint degrees, i.e. after the knee gearbox — the knee
# motor itself turns 1.5× this).
UP_KNEE_DEG = 5.0       # how much the knee opens per UP
UP_THIGH_DEG = 2.5      # thigh turns this much in the opposite direction
UP_MIN_KNEE_DEG = 15.0  # refuse UP if the knee would get straighter than this


# --- Data Models ---


class StateRequest(BaseModel):
    """JSON body of ``POST /change-state`` (all fields required)."""
    new_state: str      # SIT | STAND | UP | FORWARD | BACKWARD | LEFT | RIGHT | CW | CCW | JUMP
    sender_id: str      # who is asking (logging only)
    cycles: int         # number of gait cycles for dynamic states


# --- Core Logic ---


class QuadrupedStateManager:
    """
    Owns the robot's *intended* pose and turns state requests into packets.

    Attributes
    ----------
    current_positions : dict leg -> [x, y, z]
        Where the manager believes each foot is, as offsets (cm) from
        SIT_COORDS.  Starts at all zeros = sitting; this is an assumption —
        it does not read the robot back, so if the robot isn't actually
        sitting when the server starts, the first move will be wrong.
    static_states / dynamic_states : dict
        The recognised state names.  For static states the dict holds the
        target offset; the values in ``dynamic_states`` are placeholders.
    lock : asyncio.Lock
        Held while any state change executes.
    """
    def __init__(self):
        # Internal tracking of the current position
        self.current_positions = {leg: [0.0, 0.0, 0.0] for leg in LEG_ORDER}

        # Define Coordinate targets
        self.static_states = {
            "SIT":   {"dx": 1.5, "dy": 0.0, "dz": 0.0},
            "STAND": {"dx": 1.5, "dy": 0.0, "dz": -14.0},
        }

        self.dynamic_states = {
            "FORWARD":   {"dx": 0.0, "dy": 0.0, "dz": 0.0},
            "RIGHT":     {"dx": 0.0, "dy": 0.0, "dz": 0.0},
            "BACKWARD":  {"dx": 0.0, "dy": 0.0, "dz": 0.0},
            "LEFT":      {"dx": 0.0, "dy": 0.0, "dz": 0.0},
            "JUMP":      {"dx": 0.0, "dy": 0.0, "dz": 0.0},
            "CW": {"dx": 0.0, "dy": 0.0, "dz": 0.0},   # NEW
            "CCW":{"dx": 0.0, "dy": 0.0, "dz": 0.0},   # NEW
        }

        # The Lock ensures only one movement happens at a time
        self.lock = asyncio.Lock()

        # MPU leveler — reference is captured on SIT, correction applied on STAND
        # self.leveler = MPULeveler()


    # ─────────────────────────────────────────────────────────────────────────────
    # CORE TROT GAIT RUNNER  (translation — forward / backward / strafe)
    # ─────────────────────────────────────────────────────────────────────────────
    async def _run_trot_gait_cycle(self, axis: str, direction: int):
        """
        Always completes a full cycle before stopping — all feet land cleanly.
        axis      : 'x' fore-aft  |  'y' lateral (strafe)
        direction : +1 forward/left  |  -1 backward/right

        Timing: a cycle is 0.4 s split into 40 ticks of 10 ms.  Each tick the
        per-leg controllers give an increment, the increments are added to
        ``current_positions`` and the whole pose is sent at 500 deg/s
        (about 4× the default limit, so the gait is not slowed by the rate limiter).
        Afterwards the feet are snapped back to STAND at the default speed.
        """
        GAIT_SPEED_DEG_PER_S = 500.0
        cycle_time = 0.4
        dt = 0.01
        no_of_updates = int(cycle_time / dt)
        phase_step = 1.0 / no_of_updates
        phase = 0.0

        # Create a controller instance for each leg
        controller = {}
        for leg in LEG_ORDER:
            controller[leg] = TrotGaitController(leg, axis, direction)

        while phase < 1.0:
            payload_deltas = {}

            for leg in LEG_ORDER:
                dx, dy, dz = controller[leg]._foot_pos_phase(phase)

                # Accumulate the increment into the absolute pose …
                self.current_positions[leg][0] += dx
                self.current_positions[leg][1] += dy
                self.current_positions[leg][2] += dz

                # … and send that pose (despite the name, not a delta).
                payload_deltas[leg] = self.current_positions[leg]

            success = await self._send_to_robot_async(payload_deltas, speed=GAIT_SPEED_DEG_PER_S)
            print("\n")
            await asyncio.sleep(dt)
            phase += phase_step

        # Return all feet to STAND after the gait cycle
        target = self.static_states["STAND"]
        payload_deltas = {}
        for leg in LEG_ORDER:
            payload_deltas[leg] = [target["dx"], target["dy"], target["dz"]]
            self.current_positions[leg] = payload_deltas[leg]

        success = await self._send_to_robot_async(payload_deltas)
        phase = 0.0


    # ─────────────────────────────────────────────────────────────────────────────
    # ROTATION GAIT RUNNER  (NEW — in-place CW / CCW yaw)
    # ─────────────────────────────────────────────────────────────────────────────
    async def _run_rotation_gait_cycle(self, direction: int):
        """
        Runs one full in-place rotation cycle using RotationGaitController.
        direction : +1 → CW (viewed from above)  |  -1 → CCW

        Identical structure and timing to ``_run_trot_gait_cycle``; only the
        foot-trajectory generator differs.
        """
        GAIT_SPEED_DEG_PER_S = 500.0
        cycle_time = 0.4
        dt = 0.01
        no_of_updates = int(cycle_time / dt)
        phase_step = 1.0 / no_of_updates
        phase = 0.0

        # Create a rotation controller instance for each leg
        controller = {}
        for leg in LEG_ORDER:
            controller[leg] = RotationGaitController(leg, direction)

        while phase < 1.0:
            payload_deltas = {}

            for leg in LEG_ORDER:
                dx, dy, dz = controller[leg]._foot_pos_phase(phase)

                self.current_positions[leg][0] += dx
                self.current_positions[leg][1] += dy
                self.current_positions[leg][2] += dz

                payload_deltas[leg] = self.current_positions[leg]

            success = await self._send_to_robot_async(payload_deltas, speed=GAIT_SPEED_DEG_PER_S)
            print("\n")
            await asyncio.sleep(dt)
            phase += phase_step

        # Return all feet to STAND after the rotation cycle
        target = self.static_states["STAND"]
        payload_deltas = {}
        for leg in LEG_ORDER:
            payload_deltas[leg] = [target["dx"], target["dy"], target["dz"]]
            self.current_positions[leg] = payload_deltas[leg]

        success = await self._send_to_robot_async(payload_deltas)
        phase = 0.0


    # ─────────────────────────────────────────────────────────────────────────────
    # UP  (open knee, counter-rotate thigh → body rises)
    # ─────────────────────────────────────────────────────────────────────────────
    def _compute_up_positions(self) -> dict:
        """
        New offsets (from SIT_COORDS) for all legs after one UP step.
        Raises ValueError if any leg can't do it (nothing is changed then).

        IK knee sign: left legs θ3 > 0, right legs θ3 < 0 (``knee_dir``), so
        "open the knee" = move θ3 toward 0 by UP_KNEE_DEG, and the thigh θ2
        moves UP_THIGH_DEG the other way.
        """
        new_positions = {}
        for leg in LEG_ORDER:
            sit = SIT_COORDS[leg]
            cur = [sit[i] + self.current_positions[leg][i] for i in range(3)]
            t1, t2, t3 = leg_solver.inverse_kinematics(*cur, leg)

            knee_dir = 1 if leg in ("fl", "bl") else -1
            new_t3 = t3 - knee_dir * UP_KNEE_DEG
            new_t2 = t2 + knee_dir * UP_THIGH_DEG
            if abs(new_t3) < UP_MIN_KNEE_DEG:
                raise ValueError(
                    f"leg {leg}: knee would be {abs(new_t3):.1f}° from straight "
                    f"(limit {UP_MIN_KNEE_DEG}°) — already at max height"
                )

            x, y, z = leg_solver.forward_kinematics(
                math.radians(t1), math.radians(new_t2), math.radians(new_t3)
            )
            # Make sure the runtime's IK will accept this target.
            leg_solver.inverse_kinematics(x, y, z, leg)
            new_positions[leg] = [x - sit[0], y - sit[1], z - sit[2]]
        return new_positions

    async def _run_up(self):
        try:
            new_positions = self._compute_up_positions()
        except ValueError as e:
            raise HTTPException(status_code=400, detail=f"UP refused: {e}")

        payload = {leg: list(pos) for leg, pos in new_positions.items()}
        if not await self._send_to_robot_async(payload):
            raise HTTPException(status_code=503, detail="Robot hardware communication failed.")
        self.current_positions = new_positions
        print("[State] UP → offsets:",
              {leg: [round(v, 2) for v in p] for leg, p in new_positions.items()})
        await asyncio.sleep(0.5)   # 5° at the default speed limit takes < 0.1 s
        return {"status": "success", "reached": "UP"}


    # ─────────────────────────────────────────────────────────────────────────────
    # STATE MACHINE
    # ─────────────────────────────────────────────────────────────────────────────
    async def change_state(self, target_state: str, sender: str, cycles: int = 1):
        """
        API-like function to trigger a state transition with concurrency protection.

        Raises ``HTTPException``: 429 if a motion is already running, 400 for an
        unknown state, 503 if the static pose could not be delivered.
        Returns a small status dict (``{"status": "success", ...}``) — note the
        HTTP route currently ignores this dict (see ``handle_state_change``).

        Static state: send the pose, wait 2 s for the robot to arrive.
        Dynamic state: run the matching gait ``cycles`` times back-to-back.
        """

        # Reject (don't queue) when busy.
        if self.lock.locked():
            raise HTTPException(
                status_code=429,
                detail=f"Robot is busy. Request from {sender} rejected."
            )

        target_state = target_state.upper()
        async with self.lock:
            if target_state == "UP":
                return await self._run_up()

            if target_state not in self.static_states and target_state not in self.dynamic_states:
                raise HTTPException(status_code=400, detail=f"State '{target_state}' unknown.")

            if target_state in self.static_states:
                target = self.static_states[target_state]
                payload_deltas = {}

                for leg in LEG_ORDER:
                    payload_deltas[leg] = [target["dx"], target["dy"], target["dz"]]
                    self.current_positions[leg] = payload_deltas[leg]

                success = await self._send_to_robot_async(payload_deltas)

                if not success:
                    raise HTTPException(status_code=503, detail="Robot hardware communication failed.")

                # Give the motors time to physically get there.
                await asyncio.sleep(2.0)

                if target_state == "SIT":
                    print("[State] Bot is sitting — capturing MPU reference plane...")
                    # await asyncio.get_event_loop().run_in_executor(
                    #     None, self.leveler.capture_reference
                    # )
                    print("[State] MPU reference captured.")

                elif target_state == "STAND":
                    print("[State] Bot is standing — applying MPU level correction...")
                    # await asyncio.get_event_loop().run_in_executor(
                    #     None, self.leveler.level_after_stand
                    # )
                    print("[State] MPU level correction complete.")

                return {"status": "success", "reached": target_state}

            else:
                # Dynamic states — run the requested gait for the given number of cycles
                while cycles > 0:
                    if target_state == "FORWARD":
                        await self._run_trot_gait_cycle(axis='x', direction=1)
                    elif target_state == "BACKWARD":
                        await self._run_trot_gait_cycle(axis='x', direction=-1)
                    elif target_state == "RIGHT":
                        await self._run_trot_gait_cycle(axis='y', direction=1)
                    elif target_state == "LEFT":
                        await self._run_trot_gait_cycle(axis='y', direction=-1)
                    elif target_state == "CW":        # NEW
                        await self._run_rotation_gait_cycle(direction=+1)
                    elif target_state == "CCW":       # NEW
                        await self._run_rotation_gait_cycle(direction=-1)
                    else:
                        # e.g. JUMP: known name, no implementation.
                        return {"status": "error", "detail": f"Dynamic state '{target_state}' not implemented."}
                    cycles -= 1

                return {"status": "success", "started": target_state}


    async def _send_to_robot_async(self, payload: dict, speed: float = MAX_LIVE_DEG_PER_S):
        """
        Deliver one packet to ``main_with_ik.py``: open a TCP connection to
        SOCKET_HOST:SOCKET_PORT, send the pickled payload (legs + "speed") and
        close.  1 s connect timeout.  Returns True on success, False (after
        printing the error) on any failure — callers decide how to react.

        Note: mutates ``payload`` by adding the "speed" key.
        """
        try:
            payload["speed"] = speed
            data = pickle.dumps(payload)
            reader, writer = await asyncio.wait_for(
                asyncio.open_connection(SOCKET_HOST, SOCKET_PORT),
                timeout=1.0
            )
            writer.write(data)
            await writer.drain()
            writer.close()
            await writer.wait_closed()
            return True
        except Exception as e:
            print(f"Socket Error: {e}")
            return False


# --- FastAPI App ---


app = FastAPI(title="Quadruped API")
robot_manager = QuadrupedStateManager()


@app.post("/change-state")
async def handle_state_change(request: StateRequest):
    """
    The only route.  Awaits the full motion, then replies.

    Caveats (existing behaviour, documented rather than changed):
      * the dict returned by ``change_state`` is discarded, so "not
        implemented" (JUMP) still produces a 200 "received";
      * the response is sent only after the motion completes;
      * the log line is printed after the motion, not on receipt.
    """
    result = await robot_manager.change_state(request.new_state, request.sender_id, request.cycles)
    print(f"Received request from {request.sender_id} to change state to '{request.new_state}' for {request.cycles} cycles.")
    return {
        "sender_id": request.sender_id,
        "requested_state": request.new_state,
        "cycles": request.cycles,
        "result": "received"
    }


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
