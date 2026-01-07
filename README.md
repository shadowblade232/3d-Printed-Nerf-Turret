Controller Controls
Aiming & Movement

Left Stick

X-axis: Pan (rotate turret left/right)

Y-axis: Tilt (aim up/down)

X (hold): Slow mode
Reduces pan/tilt speed to 50% for fine aiming

Stepper Control

A: Enable steppers (pan + tilt motors)

B: Emergency stop

Disables steppers

Stops flywheels

Retracts pusher servo

Cancels homing or firing

Homing & Zeroing

LB: Set Home

Current pan/tilt position becomes 0°, 0°

RB: Go Home

Drives turret back to stored home position (0°, 0°)

Y: Auto-Home Sequence

Drives to home

Uses limit switches to find true pan & tilt limits

Automatically sets accurate home offsets

Firing System (One-Button Fire)

Right Trigger (RT):

Spins up flywheels

Pushes one dart

Retracts pusher

Spins flywheels back down
(All handled automatically)

Flywheel Velocity

D-Pad Up: Increase dart velocity

D-Pad Down: Decrease dart velocity
(Velocity is clamped to safe limits in software)

Quit Program

Hold START + BACK for ~0.6 seconds
Safely exits the program and shuts everything down

Safety & Limits

Tilt is limited to −45° to +45°

Pan wraps 0°–360°

Limit switches prevent motion into hard stops

Acceleration & deceleration are ramp-limited to avoid missed steps

Startup Behavior

On launch, the system:

Retracts the pusher servo

Lowers the popper servo

Disarms flywheels

Syncs motor position from the Pico

Sets the current position as home (0°, 0°)
