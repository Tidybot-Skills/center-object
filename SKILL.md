---
name: tb-center-object
description: Centers a detected object in the wrist camera view by moving the robot base. Use when (1) an object needs to be centered before manipulation, (2) preparing for a pick attempt, (3) aligning the camera view with a target object.
---

# Center Object

Uses YOLO detection + base velocity control to center a target object in the wrist camera frame.

## Usage

```python
from main import center_object
success, final_pos = center_object(target="banana")
success, final_pos = center_object(target="cup", tolerance=20, gain=0.002)
```

## Returns

- `(True, (u, v))` — object centered at pixel position
- `(False, None)` — failed (detection lost or max iterations)

## Configuration

| Parameter | Default | Description |
|-----------|---------|-------------|
| `target` | "banana" | YOLO class name |
| `tolerance` | 30 | Pixels from center |
| `max_iterations` | 20 | Max centering attempts |
| `gain` | 0.0015 | Movement gain (m/pixel) |
| `camera_id` | "309622300814" | Wrist camera serial |

## Notes

- Camera geometry: wrist cam looks DOWN, top of image = close to robot
- Uses base movement, not arm — preserves arm reach
- Object should be visible in wrist camera before calling

## Dependencies

None (uses robot_sdk directly).
