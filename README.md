# roxlap-game-demo

## Controls

### Translation
| Key | Action |
|-----|--------|
| W / S | Thrust up / down (body +Y / -Y) |
| A / D | Thrust left / right (body -X / +X) |
| LShift / Space | Thrust forward / backward (body -Z / +Z) |

### Rotation
| Key | Action |
|-----|--------|
| Mouse | Aim — moves the autopilot target direction |
| Q / E | Roll counter-clockwise / clockwise |

### Braking
| Key | Action |
|-----|--------|
| Tab | Hold to fire retro-thrusters — damps linear velocity (all axes) and roll |

## Autopilot

The ship continuously steers its nose toward the mouse crosshair using a bang-bang controller with a deceleration profile: it accelerates toward the target at full thrust, then begins braking early enough to stop without overshoot. Inside a small dead zone it switches to a PD controller for smooth settling.

Roll (Q/E) is independent — the autopilot ignores it and does not damp it.
