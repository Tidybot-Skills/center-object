"""
center-object skill
Centers a detected object in the wrist camera view by moving the robot base.
"""

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
    # Image center (assuming 640x480)
    CENTER_U, CENTER_V = 320, 240
    
    # Movement limits
    MAX_STEP = 0.04  # meters
    DAMPING = 0.5    # reduce oscillation
    
    def log(msg):
        if verbose:
            print(msg)
    
    log(f"[center] Starting centering for '{target}'")
    log(f"[center] Tolerance: {tolerance}px, Max iter: {max_iterations}")
    
    for iteration in range(max_iterations):
        # Capture and detect
        img = camera.capture_image(camera_id)
        detections = yolo.detect(img)
        
        # Find target
        target_det = None
        for det in detections:
            if det["label"].lower() == target.lower():
                target_det = det
                break
        
        if target_det is None:
            log(f"[center] Iter {iteration}: No '{target}' detected")
            time.sleep(0.3)
            continue
        
        # Get bounding box center
        x1, y1, x2, y2 = target_det["box"]
        u = (x1 + x2) / 2
        v = (y1 + y2) / 2
        conf = target_det.get("confidence", 0)
        
        # Calculate error from image center
        u_err = u - CENTER_U  # positive = object right of center
        v_err = v - CENTER_V  # positive = object below center
        
        log(f"[center] Iter {iteration}: pos=({u:.0f}, {v:.0f}), err=({u_err:.0f}, {v_err:.0f}), conf={conf:.2f}")
        
        # Check if centered
        if abs(u_err) < tolerance and abs(v_err) < tolerance:
            log(f"[center] SUCCESS - Object centered at ({u:.0f}, {v:.0f})")
            return True, (u, v)
        
        # Calculate base movement
        # Camera looks down: 
        #   - V error (vertical in image) -> X movement (forward/back)
        #   - U error (horizontal in image) -> Y movement (left/right)
        # Signs: negative v_err (above center) means object is close, move back (negative dx)
        #        positive u_err (right of center) means move left (positive dy)
        
        dx = -gain * v_err * DAMPING
        dy = -gain * u_err * DAMPING
        
        # Clamp to max step
        dx = max(-MAX_STEP, min(MAX_STEP, dx))
        dy = max(-MAX_STEP, min(MAX_STEP, dy))
        
        log(f"[center] Moving base: dx={dx:.4f}m, dy={dy:.4f}m")
        
        # Execute base movement
        base.move_relative(dx=dx, dy=dy, dtheta=0)
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
