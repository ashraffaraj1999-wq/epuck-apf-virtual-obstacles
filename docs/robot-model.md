# Robot Model

## 1. The e-puck

The e-puck mobile robot was designed for educational purposes at EPFL by the designers of
the Khepera robot. Its hardware and software are open source.

![e-puck components](figures/fig02-epuck-components.png)

| Component | Specification |
|---|---|
| Processor | dsPIC microcontroller, 60 MHz |
| Drive | two stepper motors |
| Wheel radius `r` | 0.0205 m |
| Axle length `l` | 0.052 m |
| Proximity sensors | 8, infrared |
| Light sensors | 8 |
| Ground sensors | 3 |
| Encoders | incremental, one per wheel |
| Camera | VGA |
| Microphones | 3, omnidirectional |
| Output | red and green LEDs, speaker (wav) |
| Communication | RS-232, Bluetooth |
| Battery | 3 Wh Li-ion, approximately 3 hours |

This project uses the wheel encoders and the motors. The proximity and light sensors are
deactivated in both controllers.

## 2. Kinematic model

The robot is driven by two motors operating differentially. Taking the goal as the origin
of an error frame:

![Robot model geometry](figures/fig09-robot-model-geometry.png)

```
ρ = √(Δx² + Δy²)
α = −θ + atan2(Δy, Δx),     α ∈ (−π/2, π/2]
β = −θ − α
```

The kinematics in these coordinates:

```
⎡ρ̇⎤   ⎡−cos α      0⎤
⎢α̇⎥ = ⎢ sin α/ρ   −1⎥ ⎡v⎤
⎣β̇⎦   ⎣−sin α/ρ    0⎦ ⎣ω⎦
```

## 3. Control law

Choosing

```
v = k_ρ · ρ
ω = k_α · α + k_β · β
```

the closed-loop state equations become

```
ρ̇ = −k_ρ · ρ · cos α
α̇ =  k_ρ · sin α − k_α · α − k_β · β
β̇ = −k_ρ · sin α
```

Near the equilibrium point the closed-loop system can be written as

```
⎡ρ̇⎤   ⎡−k_ρ      0            0  ⎤ ⎡ρ⎤
⎢α̇⎥ = ⎢  0   −(k_α − k_ρ)   −k_β ⎥ ⎢α⎥
⎣β̇⎦   ⎣  0      −k_ρ          0  ⎦ ⎣β⎦
```

Computing the eigenvalues of this matrix, it is sufficient for the roots to have strictly
negative real parts, that is, for the system to be locally stable, that

```
k_ρ > 0        k_β < 0        k_α − k_ρ > 0
```

## 4. Stability

Under the above conditions, global convergence to the unique equilibrium point `(0,0,0)`
follows from the Lyapunov function

```
V = (k_β·β)² + 2·k_β·k_ρ·(cos α − 1) + ½·ρ²
```

which is positive with a continuous first derivative on its domain. Differentiating and
substituting the state equations:

```
V̇ = 2·k_β·k_ρ·α·sin α·(k_α − (sin α/α)·k_ρ) − k_ρ·ρ²·cos α
```

For this to be strictly negative it is sufficient that

```
k_ρ > 0        k_β < 0        k_α − (sin α/α)·k_ρ > k_α − k_ρ > 0
```

Since `α ∈ (−π/2, π/2]`:

```
sin α / α < 1        cos α > 0        α·sin α > 0
```

so the equilibrium point is globally asymptotically stable.

The controllers in this project do not apply this law directly; they steer along the
potential-field resultant using `v = k_v‖F‖cos α` and `ω = k_w·α`. The analysis
establishes the convergence of a proportional heading controller of this form for a
differential-drive robot.

## 5. Odometry

Odometry estimates the robot's position relative to its initial position from the attached
encoder readings. The principal problem is the accumulation of errors over time, which
makes position prediction inaccurate after a long period of operation. Wheel slip also
affects accuracy and leads to large measurement errors.

With `Δs_r` and `Δs_l` the right and left motor angle changes during one controller
sampling period:

```
θ_new = θ_old + (r/l)·(Δs_l − Δs_r)
x_new = x_old + r·cos(θ_new)·(Δs_r + Δs_l)/2
y_new = y_old + r·sin(θ_new)·(Δs_r + Δs_l)/2
```

with `r = 0.0205 m` and `l = 0.052 m`.

As implemented in `my_controller.c`:

```c
static void compute_odometry()
{
  l = wb_position_sensor_get_value(left_position_sensor);
  r = wb_position_sensor_get_value(right_position_sensor);
  double dl = (l - prev_l);
  double dr = (r - prev_r);

  th = prev_th + WHEEL_RADIUS*(dl - dr)/AXLE_LENGTH;
  x  = prev_x  + cos(th)*WHEEL_RADIUS*(dl + dr)/2;
  y  = prev_y  + sin(th)*WHEEL_RADIUS*(dl + dr)/2;

  prev_l = l; prev_r = r; prev_th = th; prev_x = x; prev_y = y;
}
```

The initial pose is `prev_x = prev_y = 0.5` m, corresponding to the robot's start position
at the centre of the 1 × 1 m floor.

## 6. Constants in the Webots controller

```c
#define TIME_STEP           32        /* ms */
#define WHEEL_RADIUS        0.0205    /* m  */
#define AXLE_LENGTH         0.052     /* m  */
#define ENCODER_RESOLUTION  159.23
```

Motors run in velocity mode, position set to `INFINITY`, velocity commanded each step:

```c
wb_motor_set_position(left_motor, INFINITY);
wb_motor_set_velocity(left_motor, Vl/1024.0);
```

The divisor 1024 scales the controller's internal speed units to the motor's rad/s range,
corresponding to the e-puck's native speed-command full scale.
