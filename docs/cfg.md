# Comprehensive Functionality Guide

This section is intended for understanding how to modify or extend the internal structure of the system.

It describes:

- The package architecture and the data exchanges between modules.
- Key functions within each module and their customisation.

### 1. [System Architecture](about:blank#1-system-architecture)

```
SkyFall
│
├── main.py                     # Example script: run simulation and prediction
├── requirements.txt            # Python dependency list
├── README.md                   # Project overview and instructions
│
├── SkyFall/                    # Main source package
│
│   ├── simulator/
│       └── simulator.py        # Orbital dynamics + radar measurement simulation
│
│   ├── predictor/
│       └── predictor.py        # EKF-based state estimation and impact prediction
│
│   ├── visualiser/
│       └── visualiser.py       # Trajectory plotting and HTML report generation
│
│   ├── utils/
│       ├── global_variables.py    # Physical constants (G, Re, ω, etc.)
│       ├── analytical_F.py        # ∂f/∂x Jacobian for EKF state transition
│       ├── analytical_H.py        # ∂h/∂x Jacobian for EKF observation model
│       ├── predictor_utilities.py # State propagation + RK4 for EKF
│       └── __init__.py            # Python package initializer
│
│   └── __init__.py             # Python package initializer

```

| Module           | Responsibility                                               |
| ---------------- | ------------------------------------------------------------ |
| `simulator/`     | Simulates satellite motion and radar observations            |
| `predictor/`     | Applies EKF to estimate trajectory and impact site via Monte Carlo sampling |
| `visualisation/` | Generates plots and terminal outputs from prediction         |
| `utils/`         | Stores shared constants, symbolic Jacobians, etc.            |

The main script `main.py` calls these modules sequentially, handling data flow and integration logic. See tutorial for a worked example.

1. Using Individual Modules

---

### 2. `Simulator.py`:

**Simulator (Class)**

- inputs:
  - `initial_state`: np.array,
    - This specifies the starting point of the satellite from the moment it begins experiencing orbital decay.
  - `measurement_covariance`: np.array,
    - This specifies a constant matrix R. This is the noise corrupting truth of measured attributes of the satellite.
  - `timestep`: float,
    - - Timestep for the numerical method RK45.
  - `t0`: float = 0,
    - Starting point in time.
  - `t_end`: float = 4e9
    - An unreasonably large value (~127 years) on which to end the simulation. This ensures the measurements till the satellite crashes are captured.

**get_measurements (function)**

- inputs:
  - `Verbose`: Bool
    - This signposts the calling, the beginning and end of execution of the function.
- outputs:
  - `times`: np.array,
    - The times for which the ODE was numerically stepped through.
  - `measurement_noiseless`: np.array,
    - The 'real' radar station measurements (i.e. no noise).
  - `measurement_noise`: np.array,
    - The radar station measurements perturbed with additive Gaussian noise.
  - `active_radar_stations_per_time`: np.array,
    - A history of active radar stations per time step by index.
  - `active_radar_station_longitudes_per_time`: np.array,
    - A history of active radar stations' longitudes per time.
  - `crash_site`: np.array,
    - Given the tranquillity base condition on the computed standard deviation of the periodic ODE propagated Monte Carlo trajectories, this would be their mean at r=R_E.
  - `crash_time[0]`: float,
    - This is the time elapsed since the start of simulation, added by the time at which the tranquillity base condition was satisfied by the ODE propagated MC trajectories.
  - `state_noiseless.T`: np.array,
    - The trajectory of the state vector through the dynamics of the ODE system.

---

### 3. `Predictor.py`:

**Predictor (class)**

**process_model (function):** Obtain the prior state, at time $t$, mean by passing the posterior random variable, from time $t-\Delta t$ through the ODE and a sample is taken from a Gaussian random variable, 

- Inputs:
  - `posterior_state`: np.array
    - This is the posterior covariance matrix obtained from the previous iteration $\bar{P}_{t-\Delta t}$ / or the initial posterior covariance matrix $P_{0}$.
  - `process_covariance`: np.array
    - This is the process covariance matrix $Q$. This is a measure of our ‘belief’ in the physics/ in the validity of the model of the dynamics
  - `timestep`: float
    - This is the timestep used in the numerical finite difference method when evaluating the Jacobian of the continuous physics ODE dynamics, $f(.)$, $F_{t}(.)$
  - `t`: float
    - This is the time elapsed till after the last timestep was taken
  - `include_noise`: Bool
    - If true, $+Q$ else $+ 0$
  - `verbose`: Bool
    - If true, print outputs of the process model to the terminal, else don’t
- Outputs:
  - `predicted_state_with_noise`: np.array
    - $\bar{P}_t = F_t \,P_{t-\Delta t}\, F_t^T + Q$
  - `predicted_state_without_noise`: np.array
    - $\bar{P}t = F_t \,P_{t-\Delta t}\, F_t^T$

**measurement_model() (function):** Converts state into measurement space for residual calculation

- Inputs:
  - `prior_state`: np.array
    - This is the (with noise/ without noise) output of the process model
  - `theta_R`: float
    - This is the longitude of the ‘active’ satellite determined by the simulator during the satellite descent
  - `t` : float
    - The current time in the Earth-satellite system
- Outputs:
  - `h` :
    - The projection of the state vector into measurement space, $[r,\dot{r}]$

**eval_JacobianF() (function):** Computes matrix of partial derivatives for continuous ODE system, pre-calculated and stored in \utils, evaluating at current state to propagate prior state covariance

- Inputs:
  - `G, M_e, Cd, A, m, R_star, g0, M_molar, omega_E`: floats
    - See \utils for these empirically found constants
- Outputs:
  - $F_{t} = \left.\frac{\partial f(\vec{x})}{\partial \vec{x}}\right|_{ \vec{x}_{t+\Delta t}}$`eval_JacobianH()`:

**eval_JacobianH() (function):** Uses and evaluates at the predicted prior state, the symbolic Jacobians from `utils/`, to transform the vector from state-space to a lower-dimensional measurement-space representation

- Inputs:
  - `theta_R`:
    - The longitude of the ‘active’ satellite at the time self.t
  - `R_e, omega_E`: floats
    - See \utils for these empirically found constants
- Outputs:
  - $H_{t} = \left.\frac{\partial h(\vec{x})}{\partial \vec{x}}\right|_{ \vec{x}_{t}}$

**update_prior_belief() (function):** Incorporates uncertainty in equations of motion and the current state’s computed process matrix to find the covariance matrix of the predicted prior state vector

- `JacobianF`: np.array
  - Output of evaluation
- Inputs:
  - `process_covariance`: np.array
    - This is the process covariance matrix $Q$. This is a measure of our ‘belief’ in the physics/ in the validity of the model of the dynamics
  - `posterior_state_covariance`: np.array
    - $\bar{P}_t = F_t \,P_{t-\Delta t}\, F_t^T  (+ Q)$
  - `JacobianV` :
    - See functionality extension for significance
  - `verbose`: Bool
    - If True output

**kalman_gain() (function):** Kalman update step, which will act to scale the residual based on the strength of belief in the measurement

**predict_state() (function):** Uses Kalman gain matrix to combine residual with predicted prior estimate to form a mean predicted state vector and corresponding covariance matrix

**forecast() (function):** Samples are possible from the current posterior of the predicted state vector and propagate them using a numerical method to ‘project’ the current predicted posterior on Earth’s surface.

---

### 4. `Visualiser.py`

The `Visualiser` class visualises satellite orbit decay using Cartopy maps, height vs time plots, and crash longitude distributions. It supports static plots, animations, and customizable styles.

**Visualiser (Class)**

- **inputs**:
  - `times`: np.array
    - Time points (in seconds) corresponding to the satellite’s trajectory.
  - `trajectory_cartesian`: np.array
    - Satellite trajectory in Cartesian coordinates (x, y, vx, vy) in meters and meters per second.
  - `trajectory_LLA`: np.array
    - Satellite trajectory in Latitude, Longitude, Altitude (LLA) coordinates, with latitude and longitude in degrees and altitude in meters.
  - `crash_lon_list`: np.array
    - List of predicted crash longitudes (in degrees) for each forecast step.

`_adjust_crash_lon_list` (Function)

- **inputs**:
  - `crash_lon_list`: np.array
    - Array of crash longitude predictions with shape [sets, samples, dims].
  - `target_length`: int
    - Desired length of the crash longitude list, equal to the length of times.
- **outputs**:
  - `adjusted_list`: np.array
    - Array of crash longitudes adjusted to match the target length by repeating or extending prediction sets.

`plot_height_vs_time` (Function)

- **inputs**:
  - `figsize`: tuple = (8, 6)
    - Figure size (width, height) in inches.
  - `title`: str = 'Predictor altitude estimate against time'
    - Title of the plot.
  - `title_fontsize`: int = 14
    - Font size for the plot title.
  - `label_fontsize`: int = 12
    - Font size for axis labels.
  - `tick_fontsize`: int = 10
    - Font size for tick labels.
  - `line_color`: str = 'blue'
    - Color of the altitude line.
  - `line_width`: float = 2
    - Width of the altitude line.
  - `show_grid`: bool = True
    - Whether to display a grid on the plot.
  - `show_legend`: bool = True
    - Whether to display a legend.
- **outputs**:
  - None
    - Displays a plot of altitude versus time with a termination line.

`plot_orbit_map` (Function)

- **inputs**:
  - `figsize`: tuple = (8, 6)
    - Figure size (width, height) in inches.
  - `title`: str = 'Satellite Orbital Decay Trajectory'
    - Title of the plot.
  - `title_fontsize`: int = 14
    - Font size for the plot title.
  - `tick_fontsize`: int = 10
    - Font size for tick labels.
  - `path_color`: str = 'red'
    - Color of the orbit path.
  - `start_marker_color`: str = 'green'
    - Color of the start position marker.
  - `end_marker_color`: str = 'red'
    - Color of the end position marker.
  - `marker_size`: int = 10
    - Size of start and end markers.
  - `scatter_size`: int = 50
    - Size of scatter points representing altitude.
  - `cmap`: str = 'viridis'
    - Colormap for altitude scatter points.
  - `show_legend`: bool = True
    - Whether to display a legend.
- **outputs**:
  - None
    - Displays a Cartopy map showing the satellite’s orbit path with altitude-colored scatter points.

`plot_crash_distribution` (Function)

- **inputs**:
  - `figsize`: tuple = (8, 6)
    - Figure size (width, height) in inches.
  - `title`: str = 'Predicted crash site distribution'
    - Title of the plot.
  - `title_fontsize`: int = 14
    - Font size for the plot title.
  - `label_fontsize`: int = 12
    - Font size for axis labels.
  - `tick_fontsize`: int = 10
    - Font size for tick labels.
  - `box_color`: str = 'blue'
    - Color of the boxplot.
  - `show_grid`: bool = True
    - Whether to display a grid on the plot.
- **outputs**:
  - None
    - Displays a boxplot of predicted crash longitudes versus forecast index.

`plot_orbit` (Function)

- **inputs**:
  - `figsize`: tuple = (12, 10)
    - Figure size (width, height) in inches.
  - `title_fontsize`: int = 14
    - Font size for subplot titles.
  - `label_fontsize`: int = 12
    - Font size for axis labels.
  - `tick_fontsize`: int = 10
    - Font size for tick labels.
  - `map_title`: str = 'Satellite orbital decay trajectory'
    - Title for the map subplot.
  - `height_title`: str = 'Altitude against time'
    - Title for the height versus time subplot.
  - `crash_title`: str = 'Predicted crash site distribution'
    - Title for the crash distribution subplot.
  - `path_color`: str = 'red'
    - Color of the orbit path.
  - `height_line_color`: str = 'blue'
    - Color of the altitude line.
  - `box_color`: str = 'blue'
    - Color of the crash distribution boxplot.
  - `show_grid`: bool = True
    - Whether to display grids on the subplots.
  - `show_legend`: bool = True
    - Whether to display legends on the subplots.
- **outputs**:
  - None
    - Displays a 2x2 grid with a Cartopy map, altitude versus time plot, and crash longitude distribution plot.

`save_plot` (Function)

- **inputs**:
  - `filename`: str = 'orbit_decay.png'
    - Output file name for the saved plot.
  - `figsize`: tuple = (12, 10)
    - Figure size (width, height) in inches.
  - `title_fontsize`: int = 14
    - Font size for subplot titles.
  - `label_fontsize`: int = 12
    - Font size for axis labels.
  - `tick_fontsize`: int = 10
    - Font size for tick labels.
  - `map_title`: str = 'Satellite Orbit Decay Path'
    - Title for the map subplot.
  - `height_title`: str = 'Height vs Time'
    - Title for the height versus time subplot.
  - `crash_title`: str = 'Predicted Crash Longitude Distribution'
    - Title for the crash distribution subplot.
  - `path_color`: str = 'red'
    - Color of the orbit path.
  - `height_line_color`: str = 'blue'
    - Color of the altitude line.
  - `box_color`: str = 'blue'
    - Color of the crash distribution boxplot.
  - `show_grid`: bool = True
    - Whether to display grids on the subplots.
  - `show_legend`: bool = True
    - Whether to display legends on the subplots.
  - `dpi`: int = 300
    - Resolution for the saved image.
  - `bbox_inches`: str = 'tight'
    - Bounding box setting for saving the plot.
- **outputs**:
  - None
    - Saves a 2x2 grid plot (map, height, crash distribution) to the specified file.

`animate_orbit` (Function)

- **inputs**:
  - `interval`: float = 50
    - Time between animation frames in milliseconds.
  - `figsize`: tuple = (12, 12)
    - Figure size (width, height) in inches.
  - `title_fontsize`: int = 14
    - Font size for subplot titles.
  - `label_fontsize`: int = 12
    - Font size for axis labels.
  - `tick_fontsize`: int = 10
    - Font size for tick labels.
  - `map_title`: str = 'Satellite Orbital Decay Animation'
    - Title for the map subplot.
  - `height_title`: str = 'Altitude against time'
    - Title for the height versus time subplot.
  - `crash_title`: str = 'Predicted crash site distribution'
    - Title for the crash distribution subplot.
  - `path_color`: str = 'red'
    - Color of the orbit path.
  - `current_point_color`: str = 'red'
    - Color of the current position marker.
  - `height_line_color`: str = 'blue'
    - Color of the altitude line.
  - `crash_point_color`: str = 'black'
    - Color of crash site points on the map.
  - `crash_box_color`: str = 'blue'
    - Color of the crash distribution boxplot.
  - `marker_size`: int = 8
    - Size of markers for current position and start point.
  - `scatter_size`: int = 50
    - Size of scatter points for altitude.
  - `cmap`: str = 'viridis'
    - Colormap for altitude scatter points.
  - `show_grid`: bool = True
    - Whether to display grids on the subplots.
  - `show_legend`: bool = True
    - Whether to display legends on the subplots.
  - `button_pos_replay`: list = [0.45, 0.05, 0.1, 0.05]
    - Position and size of the replay button [x, y, width, height].
- **outputs**:
  - None
    - Displays an animated 2x2 grid with a Cartopy map, altitude versus time plot, and crash longitude distribution, including a replay button.

---

### 5. Utility Module

**Purpose**: Provides reusable constants, symbolic Jacobians, and helpers.

**Key Files**

- `global_variables.py`:
  - `G`:float = $6.6743×10^{-11}$ $m^3 kg^{-1}s^{-2}$
    - Gravitational constant
  - `M_e:` float = $5.972 × 10²$ kg
    - Mass of the Earth
  - `C_d`: float = 2.2
    - Drag Coefficient
  - `A`: float = $\pi (1.77/2)^2 m^2$
    - Area of the satellite
  - `m_s`: float = $479 kg$
    - Mass of the satellite
  - `R_star`: float = $8.3144598 J mol^{-1}K^{-1}$
    - Universal ideal gas constant
  - `g_0:`  float = $9.80665 m s^{-2}$
    - Gravitational acceleration at Earth’s surface
  - `M_molar`: float = $0.0289644 kg mol^{-1}$
    - Molar mass of Earth’s air
  - `omega_E`: float = $7.2921150 × 10⁻⁵ rad s^{-1}$
    - Angular rotation rate of the Earth
- `predictor_utilities.py`:
  RK-style propagation and process noise helpers.
- `analytical_F.py`, `analytical_H.py`:
  Jacobian definitions using `sympy`.

**Customisation Tips**

- Change Earth or model constants in `global_variables.py`.
- Edit symbolic expressions if model dynamics are altered.