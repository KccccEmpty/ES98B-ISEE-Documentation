# Satellite Re-entry Visualisation Demo (temp for 1D)

This module demonstrates a simple 1D satellite de-orbit simulation and visualisation workflow.

test
## Module Structure

- **`vis_test.py`** — Temporary test script. Runs the simulator, saves output, and calls the visualiser module.

- **`visualiser.py`** — Contains reusable plotting functions for:
  - Altitude vs. Time (with optional noisy measurements)
  - Velocity vs. Time
  - Trajectory (x vs y)


## What It Does

1. **Simulates satellite de-orbit** using basic physics:
   - Gravity and atmospheric drag
   - Altitude-based air density
2. **Applies Gaussian noise** to simulate radar altitude measurements
3. **Saves results** to a `.npz` file (`sim_output.npz`)
4. **Visualises**:
   - Altitude decay with/without noise
   - Horizontal and vertical velocities over time
   - Descent trajectory

## Sample Outputs
- Altitude drops from ~120 km to 0 m
- Velocity changes due to air drag
- Noisy radar measurements show uncertainty in real-world conditions

## ▶️ How to Run
Ensure you have Python 3 and required packages:
```bash
pip install numpy scipy matplotlib
```
Then run:
```bash
python vis_test.py
```
This will:
- Execute the simulation
- Save results to `sim_output.npz`
- Automatically display visual plots

## Example Plots
Plots are rendered with `matplotlib` and show:
- True vs. measured altitude
- Velocity profiles
- Descent path

## Notes
- This is based on a 1D flat-Earth simplification
- Intended for use as an early-stage simulator for larger orbital tracking projects
- Code is modular and ready to be extended with more physics (2D/3D, non-linear dynamics, EKF etc.)