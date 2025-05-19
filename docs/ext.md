# Extending Functionality

### Custom Radar-Station Placement and Visibility

This section describes in-depth how to extend the simulator and predictor to work with user-defined, geographically specific radar stations. 

- Defining Coverage Arcs:
  - `land_arcs_deg`: List[Tuple[float,float]]
    - Each tuple `(lon_start, lon_end)` defines an interval in degrees East of the equator where stations may be placed.
    - Wrap‑around across the equator: if `lon_end < lon_start`, the interval wraps via ±180°.
    - Allocating the Radar Station Counts Proportionally:
      - Compute each arc’s angular length in degrees.
      - For each span `L`, allocate the radar stations proportionally by setting `N_i = round(N_total * L/total_length)`
      - Adjusting the rounding: by letting `delta = N_total - sum(N_i)` and by adding `delta` to the arc with the largest span.
      - Uniform Station Spacing within Arcs:
        - For each arc and its allocated count `n_i`
- Uniform Radar Station Spacing within Arcs
  - This step takes each land-arc interval and its allocated station count `n_i` and places stations at equal angular intervals along the arc. For an arc from `lo` to `hi`, compute:
    `lons = np.linspace(lo, lo + ((hi - lo) % 360), n_i)`
  - The longitudes are then normalised into the $[-180 \degree, +180 \degree)$  and converted to radians for ECI calculations.
  - For further customisability, replace the linear spacing with weighted sampling (concentrating more stations in high-risk regions).
- Visibility Half-Angle $\beta$ Calculation:
  - Assuming a maximum altitude `h_star` from which a station can still see the satellite.
  - The geometry produces: $\beta = \arccos \frac{R_E}{R_E + h_{star}}$
- Active Station Selection: `active_station`
  - This function first rotates each station’s fixed longitude into the ECI frame by accounting for Earth’s rotation, then computes the smallest signed angular separation between the satellite and each station.
  - Inputs:
    - `lon_sat`: satellite longitude (rad) in ECI frame
    - `t`: simulation time (s)
  - Outputs:
    - `idx`: index of nearest station within coverage
    - `diff`: angular offset