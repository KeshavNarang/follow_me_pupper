"""Step 3: follow *me*, at a polite distance. Pure functions, no ROS.

Detections in, decisions out, so ``python3 lab/test.py step3`` checks all of
it at your desk.

Two jobs
--------
1. **How close?** The bottom edge of the box is a distance sensor
   (boxes.box_bottom). Pick the bottom edge you want -- STOP_BOTTOM, measured
   with a person standing at a comfortable distance -- and drive the error
   to zero with a proportional controller: walk in when they are far, stop
   when they are there, step back when they come too close.

2. **Which person is mine?** To YOLO every person is just class ``person``,
   so "most centred" hops between people the moment two are in frame. Two
   cues tell them apart without any new sensor or model:

   - *Where they were.* Between two frames (~0.1 s) a person barely moves
     in the image, so the person nearest where the target just was is
     almost certainly the target.
   - *What they look like.* Each detection carries the mean color of its
     segmentation mask (``detection.id``, ``"r,g,b"``) -- mostly the color
     of their clothes. A person whose color is far from the remembered one
     is a stranger, wherever they stand; and when the target vanished and
     reappears somewhere else, the color finds them again.

   A frame holding only strangers has **no** target in it: the robot searches
   for its own person rather than chasing whoever is left.
"""

import numpy as np

from boxes import IMAGE_HEIGHT, box_bottom, box_offset, detection_color

# --- Keeping the distance ---------------------------------------------------
# Box bottom edge (reference-frame px) with a person standing at the
# distance we want to keep. Measure it on the robot: stand where you would
# like Pupper to stop, save the viewer's "Live feed" image, and scale the
# y of the box's bottom edge to the 572 px frame:
#     STOP_BOTTOM = y_bottom / screenshot_height * IMAGE_HEIGHT
# ========== YOUR CODE HERE: Step 3: measure the stop distance ==========
STOP_BOTTOM = None  # px -- measure it (see above)
# =======================================================================
# Closer to STOP_BOTTOM than this and we stand still: the box jitters by a
# few pixels every frame, and a robot that shuffles back and forth to chase
# that jitter looks nervous.
STOP_TOLERANCE = 15.0
# Forward speed (m/s) per unit of bottom-edge error (error / IMAGE_HEIGHT).
KP_DISTANCE = 1.5
# Fastest the robot backs away from someone too close (m/s). Slower than
# walking forward: it cannot see where it is backing into.
MAX_BACKUP_VEL = 0.2

# --- Keeping the lock -------------------------------------------------------
# How far (in the units of position_distance: fractions of the image) a
# person may be from where the target was last frame and still count as
# "nearby". Generous enough for the robot's own turning, tight enough that a
# person across the room is not a candidate.
LOCK_GATE = 0.12
# Mask colors further apart than this (RGB distance, 0..441) are different
# people. Under the camera's fixed white balance the same shirt stays within
# a few tens of this from frame to frame.
REID_MAX_DIST = 90.0
# Per-frame blend of the remembered color toward the target's current color,
# so the memory follows a slow change in lighting instead of being shaken off
# by it.
REID_BLEND = 0.1


class Lock:
    """What we remember about the target between frames."""

    def __init__(self):
        self.position = None  # (box_offset, box_bottom / IMAGE_HEIGHT) last frame, or None
        self.color = None     # remembered mask color, a numpy array (r, g, b), or None

    def reset(self):
        """A new chase: forget everything."""
        self.position = None
        self.color = None


def box_position(detection):
    """Where a detection is, as (offset, bottom edge / IMAGE_HEIGHT).

    Both are fractions of the image: offset -0.5..+0.5, bottom 0..1.
    """
    return (box_offset(detection), box_bottom(detection) / IMAGE_HEIGHT)


def approach_speed(target_bottom, max_forward):
    """Forward speed (m/s) that holds the target at STOP_BOTTOM.

    Step 3 (distance) ----------------------------------------------------------
    target_bottom:  float, px: box_bottom of the target. Bigger = closer.
    max_forward:    float, m/s: the fastest it may walk in (follow_me.py
                    passes TRACK_FORWARD_VEL).
    Returns:        float, m/s: positive walks in, negative backs away, 0.0
                    inside STOP_TOLERANCE. Never faster than max_forward
                    forward or MAX_BACKUP_VEL backward. With max_forward 0.3:
                    at STOP_BOTTOM -> 0.0; 40 px short -> 0.105;
                    200 px short -> 0.3 (capped); bottom 572 -> -0.2
    Used by:        the TRACK state's forward speed in follow_me.py.
    Test:           python3 lab/test.py step3
    """
    # ========== YOUR CODE HERE: Step 3: approach_speed ==========
    # error = STOP_BOTTOM - target_bottom (positive = target is too far:
    # its bottom edge is above where we want it). Inside STOP_TOLERANCE,
    # stand still. Otherwise a proportional command,
    # KP_DISTANCE * error / IMAGE_HEIGHT, clamped to
    # [-MAX_BACKUP_VEL, max_forward].
    raise NotImplementedError
    # ============================================================


def position_distance(detection, position):
    """How far a detection is from a remembered position.

    Step 3 (lock) --------------------------------------------------------------
    detection:  one Detection2D.
    position:   an (offset, bottom fraction) tuple, as box_position returns
                and lock.position stores.
    Returns:    float >= 0, in fractions of the image (not pixels): 0.0 for
                the detection's own position; 70 px sideways -> 0.1
    Used by:    pick_target, against LOCK_GATE.
    Test:       python3 lab/test.py step3
    """
    # ========== YOUR CODE HERE: Step 3: position_distance ==========
    # The straight-line (Euclidean) distance between box_position(detection)
    # and position. Both are fractions of the image, so compare them directly.
    raise NotImplementedError
    # ===============================================================


def color_distance(detection, color):
    """How different a detection's mask color is from a remembered one.

    Step 3 (lock) --------------------------------------------------------------
    detection:  one Detection2D; boxes.detection_color(detection) gives its
                color as an RGB array, or None when it has none.
    color:      the remembered color (lock.color): an RGB array, or None.
    Returns:    float >= 0, the RGB (Euclidean) distance: 0.0 for the same
                shirt. None when either color is missing: that is "no
                evidence", which is not the same thing as "a match".
    Used by:    pick_target, against REID_MAX_DIST.
    Test:       python3 lab/test.py step3
    """
    # ========== YOUR CODE HERE: Step 3: color_distance ==========
    # Return None if either color is missing, else the distance between them
    # (np.linalg.norm of the difference).
    raise NotImplementedError
    # ============================================================


def pick_target(targets, lock):
    """Which of this frame's people is *our* target, or None.

    Step 3 (lock) --------------------------------------------------------------
    targets:    list of Detection2D, every detection of the tracked class in
                this frame (at least one).
    lock:       a Lock, what we remember about our target:
                  lock.position  (offset, bottom fraction) last frame, or None
                                 (a new chase, or the robot just searched)
                  lock.color     its mask color, an RGB array, or None
                Only read it here: remember updates it afterwards.
    Returns:    one of `targets`, or None when they are all strangers.
                A stranger is a person whose color_distance to lock.color is
                more than REID_MAX_DIST. (A missing color is no evidence:
                that person is not a stranger.)
    Used by:    follow_me.py's process_detections, in place of "most centred".
    Test:       python3 lab/test.py step3
    """
    # ========== YOUR CODE HERE: Step 3: pick_target ==========
    # Three rules, in order:
    #   1. Nothing remembered at all (a new chase): the most centred person.
    #   2. Where the target was is known (lock.position): of the people
    #      within LOCK_GATE of it who are not strangers, the nearest one.
    #   3. Otherwise, the person whose color is closest to lock.color, if
    #      it is within REID_MAX_DIST; else None (only strangers).
    # Tip: min(people, key=lambda d: position_distance(d, lock.position))
    # is the nearest of a non-empty list.
    raise NotImplementedError
    # =========================================================


def remember(lock, detection):
    """Update the lock after choosing ``detection`` as the target.

    Step 3 (lock) --------------------------------------------------------------
    lock:       the Lock to update, in place.
    detection:  the Detection2D pick_target just chose (never None here).
    Returns:    nothing. Afterwards lock.position == box_position(detection),
                and lock.color is the detection's color on first sight, then
                (1 - REID_BLEND) * old + REID_BLEND * new after that, so it
                follows slow lighting changes. A detection with no color
                leaves lock.color as it was.
                red [200, 40, 40], then reddish [185, 55, 50] -> [198.5, 41.5, 41]
    Test:       python3 lab/test.py step3
    """
    # ========== YOUR CODE HERE: Step 3: remember ==========
    # 1. lock.position = box_position(detection)
    # 2. color = detection_color(detection). If it is None, stop here.
    # 3. First sight (lock.color is None): store it. Otherwise blend:
    #    lock.color = (1 - REID_BLEND) * lock.color + REID_BLEND * color
    raise NotImplementedError
    # ======================================================
