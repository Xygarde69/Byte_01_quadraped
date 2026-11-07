# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  robot/state_manager_server_v3.py
# STATUS    ·  EXPERIMENT · v2 branch with MPU leveling
# ────────────────────────────────────────────────────────────────────────
# Variant of the state server that creates an MPULeveler at start-up
# (needs the MPU-6050 on I2C) so SIT captures a tilt reference and STAND
# applies a level correction. It lacks the rotation states (CW/CCW) of
# v2 and adds an unused next_state_is_walk() placeholder. Full
# documentation: state_manager_server_v2.py.
# NOTE      ·  Its STAND uses dx = 0 while mpu_leveling.py sends STAND_DX
#              = 4.
# ════════════════════════════════════════════════════════════════════════
import pickle
import socket
import asyncio
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from config.robot_config import SOCKET_HOST, SOCKET_PORT, LEG_ORDER, MAX_LIVE_DEG_PER_S
from lib.trot_gait import TrotGaitController
import time
import copy
from mpu_leveling import MPULeveler


# --- Data Models ---

class StateRequest(BaseModel):
    new_state: str
    sender_id: str
    cycles: int

# --- Core Logic ---

class QuadrupedStateManager:
    def __init__(self):
        # Internal tracking of the current position
        self.current_positions = {leg: [0.0, 0.0, 0.0] for leg in LEG_ORDER}
        
        # Define Coordinate targets
        self.static_states = {
            "SIT":   {"dx": 0.0, "dy": 0.0, "dz": 0.0},
            "STAND": {"dx": 0.0, "dy": 0.0, "dz": -14.0},
        }

        self.dynamic_states = {
            "FORWARD":  {"dx": 0.0, "dy": 0.0, "dz": 0.0},
            "RIGHT":  {"dx": 0.0, "dy": 0.0, "dz": 0.0},
            "BACKWARD": {"dx": 0.0, "dy": 0.0, "dz": 0.0},
            "LEFT":    {"dx": 0.0, "dy": 0.0, "dz": 0.0},
            "JUMP": {"dx": 0.0, "dy": 0.0, "dz": 0.0},
        }
        
        # The Lock ensures only one movement happens at a time
        self.lock = asyncio.Lock()

        # MPU leveler — reference is captured on SIT, correction applied on STAND
        self.leveler = MPULeveler()

    def next_state_is_walk(self):
        # Placeholder implementation - replace with actual logic to check the next state
        return False

    # ─────────────────────────────────────────────────────────────────────────────
    # CORE TROT GAIT RUNNER
    # ─────────────────────────────────────────────────────────────────────────────
    async def _run_trot_gait_cycle(self, axis: str, direction: int, cycles: int = 1):
        """ 
        Always completes a full cycle before stopping — all feet land cleanly.
        """
        GAIT_SPEED_DEG_PER_S = 500.0
        time = 0.4
        dt = 0.01
        no_of_updates = int(time / dt)
        phase_step = 1.0 / no_of_updates
        phase = 0.0
        
        # create a controller instance for each leg and calculate foot positions
        controller = {}  # reset controller dict each loop to ensure clean state
        for leg in LEG_ORDER:
            controller[leg] = TrotGaitController(leg, axis, direction)

        while 0.0 <= phase <= 1:
            payload_deltas = {}

            for leg in LEG_ORDER: 
                # 1. Get the ABSOLUTE position on the gait curve for this exact tick
                dx, dy, dz = controller[leg]._foot_pos_phase(phase)  

                self.current_positions[leg][0] += dx
                self.current_positions[leg][1] += dy
                self.current_positions[leg][2] += dz

                # Send the true delta directly to the physical robot
                payload_deltas[leg] = self.current_positions[leg]

            # Send the command to the physical robot hardware
            success = await self._send_to_robot_async(payload_deltas, speed=GAIT_SPEED_DEG_PER_S)
            print("\n")
            await asyncio.sleep(dt)

            phase += phase_step

        # # create a controller instance for each leg and calculate foot positions
        # controller = {}  # reset controller dict each loop to ensure clean state
        # for leg in LEG_ORDER:
        #     controller[leg] = TrotGaitController(leg, axis, direction)

        # while cycles > 0: 
        #     # check for message_queue to see if next state is also walk
        #     # if not self.next_state_is_walk():
        #     #     break
            
        #     phase = 0.5
        #     accumulated_phase = 0.0 
        #     while accumulated_phase < 1.0: 
        #         payload_deltas = {}

        #         for leg in LEG_ORDER: 
        #             # 1. Get the ABSOLUTE position on the gait curve for this exact tick
        #             dx, dy, dz = controller[leg]._foot_pos_phase(phase)  

        #             self.current_positions[leg][0] += dx
        #             self.current_positions[leg][1] += dy
        #             self.current_positions[leg][2] += dz

        #             # Send the true delta directly to the physical robot
        #             payload_deltas[leg] = self.current_positions[leg]

        #         # Send the command to the physical robot hardware
        #         success = await self._send_to_robot_async(payload_deltas, speed=GAIT_SPEED_DEG_PER_S)
        #         print("\n")
        #         await asyncio.sleep(dt)

        #         # Advance the tracking variables
        #         accumulated_phase += phase_step

        #         # Use modulo to wrap phase seamlessly from 1.0 back to 0.0
        #         phase = (phase + phase_step) % 1.0

        #     cycles -= 1

        # # create a controller instance for each leg and calculate foot positions
        # controller = {}  # reset controller dict each loop to ensure clean state
        # for leg in LEG_ORDER:
        #     controller[leg] = TrotGaitController(leg, axis, direction, "half")

        # while 0.5 <= phase <= 1.0:
        #     payload_deltas = {}

        #     for leg in LEG_ORDER: 
        #         # 1. Get the ABSOLUTE position on the gait curve for this exact tick
        #         dx, dy, dz = controller[leg]._foot_pos_phase(phase)  

        #         self.current_positions[leg][0] += dx
        #         self.current_positions[leg][1] += dy
        #         self.current_positions[leg][2] += dz

        #         # Send the true delta directly to the physical robot
        #         payload_deltas[leg] = self.current_positions[leg]

        #     # Send the command to the physical robot hardware
        #     success = await self._send_to_robot_async(payload_deltas, speed=GAIT_SPEED_DEG_PER_S)
        #     print("\n")
        #     await asyncio.sleep(dt)

        #     phase += phase_step

        target = self.static_states["STAND"]
        payload_deltas = {}
        # Calculate math inside the lock to ensure we use the latest current_positions
        for leg in LEG_ORDER: 
            payload_deltas[leg] = [target["dx"], target["dy"], target["dz"]]
            self.current_positions[leg] = payload_deltas[leg]

        # Send the command to the physical robot hardware
        success = await self._send_to_robot_async(payload_deltas)

        phase = 0.0

    async def change_state(self, target_state: str, sender: str, cycles: int = 1):
        """API-like function to trigger a state transition with concurrency protection."""
        
        # IMMEDIATE CHECK: If the robot is already moving, don't queue. Reject.
        if self.lock.locked():
            raise HTTPException(
                status_code=429, 
                detail=f"Robot is busy. Request from {sender} rejected."
            )

        target_state = target_state.upper()
        async with self.lock:
            if target_state not in self.static_states and target_state not in self.dynamic_states:
                raise HTTPException(status_code=400, detail=f"State '{target_state}' unknown.")

            if target_state in self.static_states:
                target = self.static_states[target_state]
                payload_deltas = {}
                
                # Calculate math inside the lock to ensure we use the latest current_positions
                for leg in LEG_ORDER:  
                    payload_deltas[leg] = [target["dx"], target["dy"], target["dz"]]
                    self.current_positions[leg] = payload_deltas[leg]

                # Send the command to the physical robot hardware
                success = await self._send_to_robot_async(payload_deltas)
                
                if not success:
                    raise HTTPException(status_code=503, detail="Robot hardware communication failed.")

                # Hold the lock while the legs are physically moving
                await asyncio.sleep(2.0)

                # ── SIT: capture the flat reference for the next STAND ────────
                if target_state == "SIT":
                    print("[State] Bot is sitting — capturing MPU reference plane...")
                    await asyncio.get_event_loop().run_in_executor(
                        None, self.leveler.capture_reference
                    )
                    print("[State] MPU reference captured.")

                # ── STAND: correct tilt against the captured SIT reference ────
                elif target_state == "STAND":
                    print("[State] Bot is standing — applying MPU level correction...")
                    await asyncio.get_event_loop().run_in_executor(
                        None, self.leveler.level_after_stand
                    )
                    print("[State] MPU level correction complete.")
                
                return {"status": "success", "reached": target_state}

            else:  
                if target_state == "FORWARD":  
                    await self._run_trot_gait_cycle(axis='x', direction=1)
                elif target_state == "BACKWARD": 
                    await self._run_trot_gait_cycle(axis='x', direction=-1) 
                elif target_state == "RIGHT":  
                    await self._run_trot_gait_cycle(axis='y', direction=1) 
                elif target_state == "LEFT":  
                    await self._run_trot_gait_cycle(axis='y', direction=-1) 
                else:
                    return {"status": "error", "detail": f"Dynamic state '{target_state}' not implemented."} 
                
                return {"status": "success", "started": target_state}

    async def _send_to_robot_async(self, payload: dict, speed: float = MAX_LIVE_DEG_PER_S):
        try:
            payload["speed"] = speed  # Add speed to the payload for the robot to use
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
    # This calls our manager which handles the locking logic
    result = await robot_manager.change_state(request.new_state, request.sender_id, request.cycles)
    
    return {
        "sender_id": request.sender_id,
        "requested_state": request.new_state,
        "cycles": request.cycles,
        "result": result
    }

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)