# Contents

## `webots/`

Webots R2021a project. Open `worlds/empty.wbt` to run.

| File | Description |
|---|---|
| `controllers/my_controller/my_controller.c` | Controller implementation: odometry, point-to-polygon distance, sliding-window standard deviation, APF with virtual obstacles, and the Webots main loop. |
| `controllers/my_controller/Makefile` | Webots controller makefile. |
| `worlds/empty.wbt` | World: 1 × 1 m floor, triangular obstacle, bounding wall, and an e-puck at `(0.5, 0.5)` running `my_controller`. |
| `protos/Floor3.proto` | Configurable flat floor PROTO. Copyright Cyberbotics Ltd, licensed for use only with Webots. |

Functions in `my_controller.c`:

| Function | Description |
|---|---|
| `compute_odometry()` | Estimates position and orientation from the wheel position sensors. |
| `p_poly_dist()` | Distance from the robot to the obstacle polygon, and the nearest point on it. |
| `std()` | Mean and standard deviation of the 30-sample position window. |
| `mycontroller()` | Force computation, local-minimum test, virtual obstacles, wheel speeds. |
| `main()` | Device initialisation and the `wb_robot_step` loop. |

The file contains commented-out code for an earlier approach that searched a grid around
the robot for the maximum of `F_att·(−F_rep)` to select the escape point.

## `matlab/`

| File | Description |
|---|---|
| `test_controller2.m` | Controller callback for the physical e-puck, executed on a timer by the ePic MATLAB interface. |

Required but not included:

| Dependency | Description |
|---|---|
| `p_poly_dist.m` | Point-to-polygon distance (MATLAB File Exchange). The equivalent is implemented inline in `my_controller.c`. |
| ePic toolbox | e-puck MATLAB interface providing the `ePic` object and its methods. |
| `main.m` | Interface owning the controller timer and the `ControllerState` variable. |
| `x_p.mat`, `flag.mat` | Runtime state: the position window and the local-minimum flag. |

## `docs/`

| File | Description |
|---|---|
| `algorithm.md` | Path-planning strategies, the potential field, local minima, virtual obstacles. |
| `robot-model.md` | e-puck specification, kinematic model, control law, stability analysis, odometry. |
| `implementation.md` | Differences between the MATLAB and Webots implementations. |
| `references.md` | Bibliography. |
| `figures/` | Diagrams and recorded results. |

## `docs/figures/`

| File | Description |
|---|---|
| `fig01-path-planning-with-obstacle-avoidance.jpeg` | path planning with obstacle avoidance |
| `fig02-epuck-components.png` | the e-puck and its components |
| `fig03-block-diagram.png` | project block diagram |
| `fig04-visibility-graph.png` | visibility graph method |
| `fig05-voronoi-diagram.png` | Voronoi diagram method |
| `fig06-exact-cell-decomposition.png` | exact cell decomposition |
| `fig07-approximate-cell-decomposition.png` | approximate cell decomposition |
| `fig08-potential-field-path.png` | robot path in the total potential field |
| `fig09-robot-model-geometry.png` | robot modelling geometry |
| `fig10-real-map.png` | the real map |
| `fig11-webots-world.png` | the virtual map in Webots |
| `fig12-epic-matlab-interface.png` | the ePic MATLAB interface |
| `fig13-path-without-local-minimum.png` | path without entering a potential well |
| `fig14-path-trapped-in-local-minimum.png` | path entering a potential well |
| `fig15-path-with-virtual-obstacle.png` | path with the final algorithm |

## Build output

Webots compiles the controller on first run. Object files, dependency files and
executables are not tracked; see `.gitignore`.
