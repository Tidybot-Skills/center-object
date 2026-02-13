# center-object

Centers a detected object in the wrist camera view by moving the robot base.

## Usage

```python
from main import center_object

# Center a banana in the camera view
success, final_pos = center_object(target="banana")

# With custom parameters
success, final_pos = center_object(
    target="cup",
    tolerance=20,        # pixels from center
    max_iterations=30,
    gain=0.002,          # movement gain
    verbose=True
)
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `target` | str | `"banana"` | YOLO class name to detect |
| `tolerance` | int | `30` | Pixels from center to consider "centered" |
| `max_iterations` | int | `20` | Maximum centering attempts |
| `gain` | float | `0.0015` | Movement gain (m/pixel) |
| `camera_id` | str | `"309622300814"` | Wrist camera serial |
| `verbose` | bool | `True` | Print progress |

## Returns

- `(True, (u, v))` - Object centered at pixel position
- `(False, None)` - Failed to center (detection lost or max iterations)

## How It Works

1. Captures image from wrist camera
2. Runs YOLO detection for target object
3. Calculates pixel error from image center
4. Moves base to reduce error (forward/back for V, left/right for U)
5. Repeats until within tolerance or max iterations

## Dependencies

- Robot API with base velocity control
- Wrist camera access
- YOLO model loaded on robot

## Notes

- Camera geometry: wrist cam looks DOWN (-Z), top of image = close to robot
- Centering uses base movement, not arm movement (preserves arm reach)
- Object should be visible in wrist camera before calling
