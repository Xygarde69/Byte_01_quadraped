"""
trot_gait_lib.py — EXPERIMENTAL / CURRENTLY UNUSED stateless trot generator
══════════════════════════════════════════════════════════════════════════════

Nothing in the repository imports this module (checked with grep).  The gait
that actually runs is ``lib/trot_gait.py``.  It is kept because it shows a
different design worth remembering:

    ┌─────────────────────────────┬─────────────────────────────────────────┐
    │ lib/trot_gait.py (in use)   │ lib/trot_gait_lib.py (this file)        │
    ├─────────────────────────────┼─────────────────────────────────────────┤
    │ one stateful controller per │ one *stateless* engine for all 4 legs   │
    │ leg; returns INCREMENTS     │                                         │
    │ the caller must accumulate  │ ``calculate_step_deltas`` is given the  │
    │                             │ legs' current absolute positions and    │
    │ drift if a phase is skipped │ returns the delta to the *absolute*     │
    │                             │ gait target → self-correcting           │
    └─────────────────────────────┴─────────────────────────────────────────┘

The stateless variant is more robust against dropped or repeated updates and
is a good candidate for the next refactor of the state manager.

Coordinates: cm, +X forward, Z up, relative to the sit pose (stand ≈ dz −14).
"""
import math
import time
from config.robot_config import LEG_ORDER, SIT_COORDS

class TrotGaitEngine:
    """
    Stateless diagonal-pair trot.  Tunables (set in ``__init__``):

        STRIDE_X      fore-aft travel per cycle, cm
        LIFT_Z        peak foot lift in swing, cm
        GROUND_PUSH   how far the stance foot presses down, cm
        FREQ          cycles per second (not used internally yet)
        UPDATE_HZ     intended send rate (not used internally yet)
        PHASE_OFFSETS pair A (fl, br) = 0.0 ; pair B (fr, bl) = 0.5
    """
    def __init__(self):
        # Gait Parameters (aligned to your +X=fwd, +Z=up system)
        self.STRIDE_X = 8.0      # Forward travel (cm)
        self.LIFT_Z = 5.0        # Step height (cm)
        self.GROUND_PUSH = 0.2   # Downward stance push (cm)
        self.FREQ = 2.0          # Hz
        self.UPDATE_HZ = 100.0

        self.PHASE_OFFSETS = {
            "fl": 0.0, "br": 0.0,  # Pair A
            "fr": 0.5, "bl": 0.5   # Pair B
        }

    def calculate_step_deltas(self, phase, current_positions):
        """
        Calculates dx, dy, dz relative to the CURRENT position of each leg.

        Parameters
        ----------
        phase : float in [0, 1)  — global gait phase.
        current_positions : dict leg -> [x, y, z]  — where each foot is NOW,
            as offsets from sit (the state manager's ``current_positions``).

        Returns
        -------
        dict leg -> [dx, dy, dz] to add to ``current_positions[leg]`` to reach
        this phase's absolute target.  Swing (first half): cycloidal X plus a
        raised-cosine lift; stance (second half): linear X pull-back plus a
        small downward push.
        """
        payload = {}
        for leg in LEG_ORDER:
            # phi is the local phase for this leg
            phi = (phase + self.PHASE_OFFSETS[leg]) % 1.0

            # Base height (Z) from your sitting/standing baseline
            # Since walking happens from "Stand", we use Stand's Z (-14)
            z0 = -14.0
            x0 = 0.0
            y0 = 0.0

            if phi < 0.5: # Swing Phase
                s = phi / 0.5
                # Cycloidal trajectory for Forward (X)
                dx_abs = -self.STRIDE_X/2 + self.STRIDE_X * (s - math.sin(2*math.pi*s)/(2*math.pi))
                # Lift (Z)
                dz_abs = z0 + self.LIFT_Z * (1 - math.cos(2*math.pi*s))
            else: # Stance Phase
                s = (phi - 0.5) / 0.5
                # Linear pushback for Forward (X)
                dx_abs = self.STRIDE_X/2 - self.STRIDE_X * s
                # Small ground push (Z)
                dz_abs = z0 - self.GROUND_PUSH * math.sin(math.pi * s)

            # Calculate relative delta: (Target_Absolute - Current_State_Manager_Absolute)
            # This ensures main_with_ik gets the correct relative shift[cite: 4, 8]
            dx = dx_abs - current_positions[leg][0]
            dy = y0 - current_positions[leg][1]
            dz = dz_abs - current_positions[leg][2]

            payload[leg] = [dx, dy, dz]

        return payload
