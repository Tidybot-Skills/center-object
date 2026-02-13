"""
center-object skill
Centers a detected object in the wrist camera view by moving the robot base.
Includes rotation search (±20°) when object is not initially visible.
"""

from robot_sdk import base, yolo
import math
import time


def center_object(
    target="banana",
    tolerance=30,
    max_iterations=20,
    gain=0.0015,
    camera_id="309622300814",
    verbose=True
):
    """
    Center a target object in the wrist camera by moving the base.
    
    Args:
        target: YOLO class name to detect
        tolerance: Pixels from center to consider "centered"
        max_iterations: Maximum centering attempts
        gain: Movement gain (meters per pixel of error)
        camera_id: Wrist camera serial number
        verbose: Print progress messages
    
    Returns:
        (success: bool, position: tuple or None)
        - (True, (u, v)) if object centered at pixel position
        - (False, None) if failed
    """
    CENTER_U, CENTER_V = 320, 240
    MAX_STEP = 0.04
    DAMPING = 0.5
    def log(msg):
        if verbose:
            print(msg)
    
    def detect_target():
        result = yolo.segment_camera(target, camera_id=camera_id, confidence=0.15)
        for det in result.detections:
            if det.class_name.lower() == target.lower():
                return det
        return None
    
    def rotational_center(det):
        """After finding object, rotate more to center it horizontally (U axis)"""
        U_TOLERANCE = 80  # pixels from center
        FINE_ANGLE = math.radians(10)  # 10° fine rotation steps
        MAX_FINE_ROTATIONS = 6
        
        for i in range(MAX_FINE_ROTATIONS):
            x1, y1, x2, y2 = det.bbox
            u = (x1 + x2) / 2
            u_err = u - CENTER_U
            
            if abs(u_err) < U_TOLERANCE:
                log(f"[center] Rotationally centered (u_err={u_err:.0f})")
                return det
            
            # Rotate towards center
            if u_err > 0:  # Object is right of center, rotate left (negative)
                log(f"[center] Object right of center (u_err={u_err:.0f}), rotating -10°")
                base.move_delta(dtheta=-FINE_ANGLE)
            else:  # Object is left of center, rotate right (positive)
                log(f"[center] Object left of center (u_err={u_err:.0f}), rotating +10°")
                base.move_delta(dtheta=FINE_ANGLE)
            
            time.sleep(0.3)
            det = detect_target()
            if det is None:
                log(f"[center] Lost detection during rotational centering")
                return None
        
        log(f"[center] Rotational centering done (max steps)")
        return det
    
    def search_rotate():
        """Rotate base ±30° then ±60° to search for object, then rotationally center"""
        angle_30 = math.radians(30)
        
        log(f"[center] Searching: rotating +30°...")
        base.move_delta(dtheta=angle_30)
        time.sleep(0.3)
        det = detect_target()
        if det:
            log(f"[center] Found at +30°, centering rotationally...")
            return rotational_center(det)
        
        log(f"[center] Searching: rotating -60° (to -30°)...")
        base.move_delta(dtheta=-2*angle_30)
        time.sleep(0.3)
        det = detect_target()
        if det:
            log(f"[center] Found at -30°, centering rotationally...")
            return rotational_center(det)
        
        base.move_delta(dtheta=angle_30)
        time.sleep(0.2)
        
        angle_60 = math.radians(60)
        
        log(f"[center] Searching: rotating +60°...")
        base.move_delta(dtheta=angle_60)
        time.sleep(0.3)
        det = detect_target()
        if det:
            log(f"[center] Found at +60°, centering rotationally...")
            return rotational_center(det)
        
        log(f"[center] Searching: rotating -120° (to -60°)...")
        base.move_delta(dtheta=-2*angle_60)
        time.sleep(0.3)
        det = detect_target()
        if det:
            log(f"[center] Found at -60°, centering rotationally...")
            return rotational_center(det)
        
        log(f"[center] Not found after ±30° and ±60° search")
        base.move_delta(dtheta=angle_60)
        time.sleep(0.3)
        return None
    
    log(f"[center] Starting centering for '{target}'")
    log(f"[center] Tolerance: {tolerance}px, Max iter: {max_iterations}")
    
    # Initial detection
    target_det = detect_target()
    
    # If not found, do rotation search
    if target_det is None:
        log(f"[center] Object not visible, starting rotation search...")
        target_det = search_rotate()
        if target_det is None:
            log(f"[center] FAILED - Object not found after search")
            return False, None
    
    consecutive_misses = 0
    wiggle_step = 0.02  # 2cm
    
    for iteration in range(max_iterations):
        result = yolo.segment_camera(target, camera_id=camera_id, confidence=0.15)
        detections = result.detections
        
        target_det = None
        for det in detections:
            if det.class_name.lower() == target.lower():
                target_det = det
                break
        
        if target_det is None:
            consecutive_misses += 1
            log(f"[center] Iter {iteration}: No '{target}' detected ({consecutive_misses}/10)")
            
            if consecutive_misses >= 10:
                log(f"[center] FAILED - 10 consecutive detection failures")
                return False, None
            
            # Wiggle base slightly to recover detection
            dx = wiggle_step * (1 if consecutive_misses % 2 == 0 else -1)
            dy = wiggle_step * (1 if (consecutive_misses // 2) % 2 == 0 else -1)
            log(f"[center] Wiggling base: dx={dx:.3f}m, dy={dy:.3f}m")
            base.move_delta(dx=dx, dy=dy)
            time.sleep(0.3)
            continue
        
        consecutive_misses = 0  # Reset on successful detection
        
        # Get bbox center
        x1, y1, x2, y2 = target_det.bbox
        u = (x1 + x2) / 2
        v = (y1 + y2) / 2
        conf = target_det.confidence
        
        u_err = u - CENTER_U
        v_err = v - CENTER_V
        
        log(f"[center] Iter {iteration}: pos=({u:.0f}, {v:.0f}), err=({u_err:.0f}, {v_err:.0f}), conf={conf:.2f}")
        
        if abs(u_err) < tolerance and abs(v_err) < tolerance:
            log(f"[center] SUCCESS - Object centered at ({u:.0f}, {v:.0f})")
            return True, (u, v)
        
        dx = -gain * v_err * DAMPING
        dy = -gain * u_err * DAMPING
        
        dx = max(-MAX_STEP, min(MAX_STEP, dx))
        dy = max(-MAX_STEP, min(MAX_STEP, dy))
        
        log(f"[center] Moving base: dx={dx:.4f}m, dy={dy:.4f}m")
        
        base.move_delta(dx=dx, dy=dy)
        time.sleep(0.3)
    
    log(f"[center] FAILED - Max iterations ({max_iterations}) reached")
    return False, None


# Allow running directly for testing
if __name__ == "__main__":
    success, pos = center_object(target="banana", verbose=True)
    if success:
        print(f"Centered at {pos}")
    else:
        print("Centering failed")
