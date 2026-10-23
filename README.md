# Pioneer P3DX Obstacle Avoidance in CoppeliaSim

Reactive obstacle avoidance for a Pioneer P3DX robot in CoppeliaSim, controlled entirely from Python over the ZeroMQ Remote API. The robot navigates a cluttered arena using only its 16 ultrasonic proximity sensors. No child script runs on the robot.


## Features

- **Finite state machine** (`FORWARD` → `BACKUP` → `TURN`) running in synchronous stepping mode, so sensing and actuation are deterministic at every simulation step.
- **Bearing from detection axis.** Each sensor's bearing is computed from its local z-axis (the direction it actually detects), not from where it is mounted. Readings are grouped into front, side and rear zones.
- **Startup self-calibration.** The robot drives forward briefly to find its true forward axis, then spins in place to find its turn polarity. This removes assumptions about the model's frame orientation and wheel wiring.
- **Closed-loop turning.** Each avoidance turn rotates a fixed `TURN_ANGLE` (20° by default), measured from the robot's heading rather than timed.
- **Speed scaling and side steering.** Forward speed drops as the front range shrinks, and a small differential steer moves the robot away from nearby side walls.
- **Floor-edge protection.** Ultrasonic sensors cannot see the edge of a finite floor. The script reads the floor's bounding box and ray-casts to its edges as virtual walls, with a z-height check that stops the run if the robot falls.
- **Automatic detectability.** Obstacle shapes are flagged as detectable at startup so the sensors can see them.

## Requirements

- CoppeliaSim (tested with the Edu version)
- Python 3.9 or newer
- The CoppeliaSim ZeroMQ Remote API client:

```bash
pip install coppeliasim-zmqremoteapi-client
```

## Usage

1. Open your scene in CoppeliaSim with a Pioneer P3DX and the obstacles. Make sure the ZeroMQ Remote API server is running (port `23000` by default).
2. Remove or disable any child script attached to the Pioneer, since it will fight with the Python wheel commands.
3. Run the script:

```bash
python main.py
```

On startup you should see the motors, robot base and sensors found, the floor bounds, and the calibration results. The console then prints a status line every second:

```
[  3.00s] FORWARD  front=0.84 left=inf right=1.20 rear=inf hits=2
```

Press `Ctrl+C` to stop. The script halts the wheels and stops the simulation on exit.

## How it works

| State | Behaviour |
|---|---|
| `FORWARD` | Drives ahead, slowing as the front range shrinks and steering gently away from close side walls. Switches to `TURN` when the front range drops below `STOP_DISTANCE`, or to `BACKUP` if it is closer than `CRITICAL_DISTANCE`. |
| `BACKUP` | Reverses for up to `REVERSE_TIME`, or until the rear range falls below `REAR_CLEARANCE`. |
| `TURN` | Spins in place until the heading has changed by `TURN_ANGLE`, then returns to `FORWARD`. If blocked again within `KEEP_DIRECTION_TIME`, it keeps turning the same way to avoid flip-flopping. |

The turn direction is chosen toward the side with more free space.

## Configuration

All settings are at the top of `main.py`.

| Setting | Default | Description |
|---|---|---|
| `FORWARD_SPEED` | `2.0` | Wheel speed (rad/s) when driving ahead |
| `TURN_SPEED` | `1.0` | Wheel speed (rad/s) while turning. Lower values overshoot less |
| `TURN_ANGLE` | `20` | Degrees rotated per avoidance turn |
| `STOP_DISTANCE` | `0.25` | Start avoiding when something is this close ahead (m) |
| `SLOW_DISTANCE` | `0.70` | Begin slowing down inside this range (m) |
| `CRITICAL_DISTANCE` | `0.12` | Back up first if this close. Keep it below `STOP_DISTANCE` |
| `SIDE_DISTANCE` | `0.15` | Steer away from walls closer than this on the sides (m) |
| `REAR_CLEARANCE` | `0.20` | Do not reverse if something is closer than this behind (m) |
| `EDGE_OFFSET` | `0.30` | Distance from the floor edge at which the robot reacts (m) |
| `MANUAL_ARENA` | `None` | Set `(xmin, xmax, ymin, ymax)` in world metres if automatic floor detection fails |
| `MAX_SIM_TIME` | `120` | Simulation time limit (s) |

## Troubleshooting

- **All sensors read `inf` (`hits=0`) while touching a wall.** The obstacles are not detectable. The script tries to fix this automatically. If it reports failures, open each Cuboid, go to the Common tab and enable the ultrasonic Detectable option.
- **"Robot barely moved during calibration."** Another script is probably controlling the wheels. Disable the Pioneer's child script.
- **"Floor not found."** Set `MANUAL_ARENA` so the edge protection works.
- **Robot hits walls.** Increase `STOP_DISTANCE` or lower `FORWARD_SPEED`.
- **Turns too much or too little.** Adjust `TURN_ANGLE`. The robot can overshoot by a degree or two because it keeps spinning briefly after the wheels stop, so a slightly smaller value gets closer to the target.

## Project structure

```
.
├── main.py      # Navigation script
└── README.md
```

## Possible extensions

- Occupancy-grid mapping and path planning (for example A*)
- Deployment on real hardware

## License

Add a license of your choice (for example MIT) as a `LICENSE` file.
