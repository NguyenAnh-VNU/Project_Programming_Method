# Ball & Plate — 3D PID Simulator

A real-time 3D physics simulation of a ball-and-plate balancing system, controlled by a dual-axis PID controller. Built with Python and Pygame, the simulation features a tilting circular plate mounted on a 3-legged tripod, a physics-accurate bouncing ball, and a fully interactive control panel.

---

## Features

- **Dual-axis PID control** — independent X and Y PID controllers keep the ball balanced at a target point
- **Realistic ball physics** — gravity, rolling friction, velocity cap, bounce restitution, and airborne drift
- **3D rendering** — isometric projection with free camera (rotate, pan, zoom)
- **Orbit mode** — ball tracks a circular orbit path; radius and speed are adjustable
- **Push mode** — drag-to-aim impulse system lets you flick the ball in any direction
- **Interactive sliders** — tune Kp, Ki, Kd, bounce restitution, push parameters, and orbit settings live
- **Live error graphs** — real-time X/Y position error plots displayed in the side panel
- **Tripod visual** — animated 3-legged mount with joint highlights reacts to plate tilt

---

## Requirements

- Python 3.8+
- Pygame 2.x

Install dependencies:

```bash
pip install pygame
```

---

## Running the Simulation

```bash
python PID-Ball-Controller.py
```

---

## Controls

| Input | Action |
|---|---|
| **Left-click** (simulation area) | Place / drag the ball |
| **Right-click drag** | Rotate camera |
| **Shift + Right-click drag** | Pan camera |
| **Scroll wheel** | Zoom in / out |
| **U** | Toggle Push mode (aim & flick the ball) |
| **O** | Toggle Orbit mode (ball follows a circular path) |
| **R** | Reset simulation (ball, plate tilt, PID state, drop counter) |
| **C** | Reset camera to default view |

In **Push mode**, left-click and drag away from the ball to aim, then release to apply an impulse. The direction and drag length determine the push direction and power.

---

## PID Tuning Sliders

| Slider | Range | Default | Description |
|---|---|---|---|
| **Kp** | 0.001 – 0.30 | 0.08 | Proportional gain |
| **Ki** | 0.000 – 0.005 | 0.0005 | Integral gain |
| **Kd** | 0.000 – 1.0 | 0.35 | Derivative gain (EMA-filtered) |
| **Bounce** | 0.0 – 1.0 | 0.62 | Ball restitution coefficient |
| **Push Max Speed** | 30 – 350 | 150 | Maximum impulse velocity |
| **Push Sensitivity** | 0.5 – 4.0 | 1.5 | Drag-to-speed sensitivity |
| **Push Scale** | 0.5 – 3.0 | 1.2 | Global push power multiplier |
| **Orbit Radius** | 40 – 190 | 160 | Radius of the orbit target path (px) |
| **Orbit Speed** | 0.1 – 3.0 | 0.6 | Angular speed of orbit (rad/s) |

---

## Architecture

```
PID-Ball-Controller.py
│
├── Camera          — Handles 3D projection, mouse rotation, pan, zoom
├── PIDController   — EMA-filtered derivative, clamped integral, dual-axis
├── Ball            — Physics update (on-plate & airborne), bounce, trail
├── Plate           — Tilt slew-rate limiting, impact wobble, angular velocity
├── UIWidget        — Base class for UI elements
│   ├── Slider      — Draggable value control
│   └── ErrorGraph  — Scrolling real-time error plot
├── ToggleMode      — Base for on/off mode controllers
│   ├── PushAimer   — Drag-to-aim impulse system
│   └── OrbitController — Circular setpoint orbit
└── App             — Main loop, rendering, event dispatch, PID integration
```

---

## Key Implementation Notes

- **Derivative kick fix** — The PID derivative term uses an exponential moving average (α = 0.25) to smooth out the large spike that occurred when `pid.reset()` was called between modes.
- **Integral windup** — The integral is clamped to ±400 (tightened from ±600) to reduce overshoot after disturbances.
- **Airborne drift** — While the ball is bouncing, a fraction (`AIR_DRIFT = 0.18`) of the plate-tilt gravity is still applied so the PID can steer the ball back toward the target mid-air.
- **Restitution as parameter** — The bounce coefficient is no longer a mutated global; it is passed directly into `Ball.update()` from the slider value each frame.
- **Per-frame trig caching** — Plate tilt sin/cos values are computed once per frame in `_cache_tilt()` and reused across all rendering and physics calls.
- **Squared boundary check** — Ball off-plate detection uses a squared distance comparison to avoid a `sqrt` call every frame.

---

## Project Structure

```
.
├── PID-Ball-Controller.py   # Main simulation file
├── README.md                # This file
└── State_Diagram.uml        # UML startup sequence diagram
```
