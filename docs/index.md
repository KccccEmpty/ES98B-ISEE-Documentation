# Introduction
This project provides a **satellite orbital decay simulation and a prediction system**, designed to model and predict the trajectory of a satellite (ISEE-3) entering Earth’s atmosphere along an equatorial orbit.

### Primary objectives:

- Assist **users** in setting up the environment, indicate reasonable use cases and how to launch and vary the simulation to produce visualisations.
- Guide **developers** to understand how to extend the functionality for advanced use cases.

### Key package features:

- Configurable orbital decay simulation based on popular physical models and assumptions:
- Spherical Earth, uniform density, constant rotation.
- Time-invariant satellite surface area, non-rotating around any of its axes.
- U.S. Standard Atmosphere 1976 model.
- Isolated system (Earth and satellite).
- Radar-based state estimation using a configurable **Extended Kalman Filter (EKF)**.

### Limitations & current known issues

- A spherical Earth with uniform density is assumed.
- Assumes constant surface area and a non-tumbling satellite.
- Assumes the drag equation is valid from start to end - does not consider non-continuum effects above 86km.
- Constant radar noise model (no occlusion or range bias).
- EKF-based filtering (prone to numerical instability).
- Uncertainty quantification based on Monte Carlo sampling; computationally expensive for large sample sizes.
