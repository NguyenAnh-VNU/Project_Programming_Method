# Ball & Plate — 3D PID Simulation
A real-time 3D simulation of a ball-balancing system controlled by a PID algorithm. Built with Python and Pygame.

![Python](https://img.shields.io/badge/Python-3.11+-blue?logo=python)
![Pygame](https://img.shields.io/badge/Pygame-2.x-green?logo=pygame)
![License](https://img.shields.io/badge/License-MIT-yellow)

## Features
- **Dual-axis PID Controller** — keeps the ball centred on the plate automatically
- **Push Mode** — click-drag to aim and launch the ball with custom power
- **Orbit Mode** — PID tracks a moving circular setpoint
- **Drag & Drop** — left-click to place the ball anywhere in 3D space
- **Live Tuning** — real-time sliders for Kp, Ki, Kd, bounce restitution, orbit radius/speed
- **3D Free Camera** — right-click drag to orbit, Shift+drag to pan, scroll to zoom
- **Error Graphs** — real-time scrolling X/Y PID error display
- **Tripod Actuators** — animated piston legs visualise plate tilt

## OOP Design
```
SimObject (ABC)
├── ControlObject (ABC)  — reset() contract
│   ├── PIDController    — discrete-time PID with anti-windup
│   └── OrbitController  — circular setpoint generator
├── UIObject (ABC)       — handle() + draw() contract
│   ├── Slider           — draggable parameter widget
│   └── ErrorGraph       — real-time scrolling graph
├── Ball                 — physics body with state machine
└── Plate                — tilting platform with angular dynamics

Camera / PushAimer / App / BallState
```

- **Encapsulation** — private fields (`_integral`, `_prev_error`, `_dragging`) with clean public APIs
- **Inheritance** — `PIDController` and `OrbitController` extend `ControlObject`; `Slider` and `ErrorGraph` extend `UIObject`
- **Modular design** — each class has a single responsibility; swap or remove any without touching others

## Ball State Diagram
```
   ┌──────────┐  LMB click    ┌──────────────┐  LMB release   ┌─────────────┐
   │   IDLE   │ ────────────► │   DRAGGING   │ ─────────────► │   AIRBORNE  │ ◄──┐
   └──────────┘               └──────────────┘                └──────┬──────┘    │
        ▲                                                            │           │
        │                                                    z ≤ plate_z ?       │
        │                                        ┌───────────────────┤           │
        │                                 impact < MIN          impact ≥ MIN     │
        │                                        │                   │           │
        │                                        ▼                   ▼           │
        │                                 ┌──────────┐        ┌──────────────┐   │
        │                                 │ ON_PLATE │        │   BOUNCING   │───┘
        │                                 └────┬─────┘        └──────────────┘
        │                          ┌───────────┴───────────┐
        │                   [O] key pressed         dist > PLATE_SIZE
        │                          │                       │
        │                          ▼                       ▼
        │                  ┌───────────────┐         ┌──────────┐
        │                  │ORBIT_TRACKING │         │   DEAD   │
        │                  └───────┬───────┘         └────┬─────┘
        │                  dist > PLATE_SIZE         auto-reset
        │                          |                      |
        |                          ▼                      │
        └─────────────────────[ DEAD ] ◄──────────────────┘
```

See `State_Diagram.svg` for the full rendered diagram.


## PID Controller
```
output = Kp × error  +  Ki × ∫error dt  +  Kd × d(error)/dt
```

- **Kp** — correction strength
- **Ki** — eliminates steady-state offset; clamped ±400 to prevent windup
- **Kd** — damps oscillation
- Output clamped to `±22°` before being sent to the plate


## Installation
```bash
git clone https://github.com/your-username/project_programming_2.git
cd project_programming_2
pip install pygame
python "PID-Ball-Controller.py"
```

## Controls
| Key | Action |
|---|---|
| Left-click | Place / drag ball |
| U | Toggle push mode |
| O | Toggle orbit mode |
| R | Reset simulation |
| C | Reset camera |
| Right-click drag | Orbit camera |
| Scroll | Zoom |

## Project Structure
```
project_programming_2/
├── PID-Ball-Controller.py    # Main simulation
├── State_Diagram.svg    # Ball state machine
├── State_Diagram.puml   # PlantUML source
├── README.md
└── requirements.txt
```

## Git Commit Convention
```
feat:     new feature
fix:      bug fix
refactor: code restructuring
docs:     documentation
```

Example commits:
```
feat: add BallState enum for deterministic lifecycle
refactor: extract TripodRenderer from App
fix: clamp PID integral to prevent windup
docs: add state diagram and README
```

## License
MIT License — see [LICENSE](LICENSE) for details.