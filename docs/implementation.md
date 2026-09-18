# Implementation Notes

Two implementations of the algorithm exist: a MATLAB controller for the physical e-puck
and a C controller for the Webots simulation.

| | `matlab/test_controller2.m` | `webots/.../my_controller.c` |
|---|---|---|
| Target | physical e-puck over serial | Webots R2021a |
| Language | MATLAB, timer callback | C, Webots controller |
| Step time | set by the host timer | `TIME_STEP = 32` ms |
| Position source | `get(ePic,'odom')` | `compute_odometry()` from encoders |
| Polygon distance | `p_poly_dist()` (external) | implemented inline |

## 1. Per-step sequence

Both follow the same sequence:

```
1.  update pose from odometry
2.  compute distance and nearest point on the obstacle polygon
3.  F_att = k_a · (goal − pos)
4.  F_rep = FIRAS function, if within ρ₀
5.  push pose into the sliding position window; compute σ
6.  if local-minimum conditions are met, add a virtual obstacle
7.  F_ext = sum of repulsions from the virtual obstacles
8.  F = F_att + F_rep + F_ext
9.  α = atan2(Fy,Fx) − θ ;  v = k_v‖F‖cos α ;  ω = k_w·α
10. write wheel speeds
```

## 2. Differences

### Local-minimum condition

MATLAB uses the position-window spread and distance conditions:

```matlab
if norm(t_std) <= 1.4/1.2 && norm(pos - goal) > 9 && norm(pos - start) > 5
```

The Webots version adds the angle between the attractive and repulsive forces:

```c
double dot = fx_a*fx_r + fy_a*fy_r;
angle = acos(dot/(fa_norm*fr_norm)) * 180/π;
...
if (norm_std <= 0.05 && goal_dist > 9 && start_dist > 3
    && (counter >= wait_time || first_virt) && angle > 90)
```

A small position spread alone indicates only that the robot is moving slowly. An angle
greater than 90° indicates the two forces are opposing each other, which is the condition
that produces the well.

### Number of virtual obstacles

MATLAB maintains a single escape point (`xtp`, `ytp`), replaced on each detection. The
Webots version maintains an array and sums the repulsion from all entries:

```c
double virtual[50][2] = {0};
int numvirt = 0;
...
virtual[numvirt][0] = meanx;  virtual[numvirt][1] = meany;  numvirt++;
```

### Scaling near the goal

```c
fext_x *= (1 - exp(-pow(goal_dist,2)/(sigma)));
fext_y *= (1 - exp(-pow(goal_dist,2)/(sigma)));
```

The factor approaches zero as the robot nears the goal, so a virtual obstacle placed near
the goal does not oppose the attractive force there. The MATLAB version contains the same
expression but it is not applied to the virtual force.

### Re-trigger guard

`wait_time = 120` steps must elapse before a further virtual obstacle can be added, unless
it is the first. This prevents repeated detections while the robot is still leaving the
well.

## 3. Differences in convention between the two files

**Wheel-speed assignment**

| Source | Left | Right |
|---|---|---|
| Report | `v_l = v + ω` | `v_r = v − ω` |
| Webots C | `Vl = V + W` | `Vr = V - W` |
| MATLAB | `vl = V - W` | `vr = V + W` |

**Obstacle vertex arrays**

```matlab
% MATLAB
xv = [22 26  7 22];
yv = [ 9 25 21  9];
```

```c
/* Webots C */
const double yv[] = {22+46, 26+46,  7+46, 22+46};
const double xv[] = { 9+46, 25+46, 21+46,  9+46};
```

The two arrays hold the same values with x and y exchanged. The goal position is not
exchanged (`xg = 36`, `yg = 24.5` in both, offset by 46 in the C version). Webots R2021a
uses an NUE coordinate system, in which the horizontal plane of the world maps to the x
and z axes.

**Coordinate offset**

The Webots floor is 1 × 1 m with its origin at a corner and the robot starts at the centre
`(0.5, 0.5) m = (50, 50) cm`. The MATLAB run resets odometry to `[0.04 0.04 0] m = (4, 4)
cm`. The constant `46 = 50 − 4` maps the map coordinates into the Webots world frame.

## 4. Point-to-polygon distance

`p_poly_dist()` in the C file computes the shortest distance from the robot to the obstacle
polygon and the corresponding nearest point.

For each edge it forms the line `Ax + By + C = 0`, projects the robot position onto it, and
tests whether the projection lies within the segment:

```c
if (((xp >= xv[i-1]) && (xp <= xv[i])) || ((xp >= xv[i]) && (xp <= xv[i-1])))
```

If so, that perpendicular distance is a candidate. The nearest polygon vertex is tracked
separately. The result is whichever is smaller:

```c
if (min_v <= min_p) { d = min_v; x_poly = xv[indx_v]; y_poly = yv[indx_v]; }
else                { d = min_p; x_poly = xp_min;     y_poly = yp_min;     }
```

The vertex case is required near a corner, where the perpendicular to every edge falls
outside its segment.

The loop runs `for (i = 1; i <= 3; i++)` over a four-element array whose last element
repeats the first, closing the triangle.

## 5. State persistence in the MATLAB controller

The MATLAB controller is a timer callback: it returns after each step and retains only
`persistent` variables. The position window and the local-minimum flag are therefore
stored on disk between calls:

```matlab
load('x_p.mat','t')
load('flag.mat','local_minima_Flag')
...
save('flag.mat','local_minima_Flag');
save('x_p.mat','t');
```

The window length is determined by the contents of `x_p.mat`. The C version uses a
file-scope array with an explicit window length of 30.

## 6. Controller state machine (MATLAB)

`test_controller2.m` plugs into the ePic interface's state machine through the global
`ControllerState`:

| Value | Meaning |
|---|---|
| `0` | controller off |
| `1` | transition to on in progress; initialisation code runs |
| `-1` | transition to off in progress; termination code runs |
| `-2` | suspended, control goal reached |
| other | controller running |

Initialisation deactivates the accelerometer, proximity and light sensors and resets
odometry to `[0.04 0.04 0]`.
