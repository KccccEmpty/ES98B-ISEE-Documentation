# Environment & Dependencies

### Environment setup

To ensure smooth execution and modular usage of the project, please follow the environment setup guidelines below. The system should be compatible across platforms, but this has not been rigorously tested.

---

**Recommended Global Environment**

| Item             | Recommended Configuration      |
| ---------------- | ------------------------------ |
| Python Version   | Python ≥ 3.10                  |
| Operating System | Windows 10/11, MacOS, or Linux |
| Package Manager  | `pip` or `conda`               |

### Module-wise dependencies

The project is designed with a modular, class-based architecture, allowing individual components (e.g., the simulator, predictor, or visualiser) to be executed or extended independently.

We explicitly state the dependency versions to ensure long-term maintainability, prevent future compatibility issues, and support modular reuse or integration in other systems.

| Module         | Path                     | Python Version | Notes                                                        |
| -------------- | ------------------------ | -------------- | ------------------------------------------------------------ |
| **Simulator**  | `SkyFall/simulator/`     | **≥ 3.10**     | Uses `NumPy`, `SciPy`; any version ≥3.10 is sufficient for numerical solvers |
| **Predictor**  | `SkyFall/predictor/`     | **3.x**        | EKF, compatible with any modern Python 3 version             |
| **Visualiser** | `SkyFall/visualisation/` | **≥ 3.10**     | Uses `matplotlib`, `cartopy` (include link); tested with 3.10+ |
| **Utils**      | `SkyFall/utils/`         | –              | Shared functions, no specific requirements                   |

### Package-wise dependencies

This project relies on several third-party Python packages for numerical integration, state estimation, visualisation and much more. Below is a complete list of dependencies used across all modules, along with module-specific usage notes and special considerations.

**Required Packages**

To use SkyFall, the following third-party packages are required. For the full list of required dependencies and their versions, please see `requirements.txt`. 

| Package      | Purpose                                      | Used In Modules                                  |
| ------------ | -------------------------------------------- | ------------------------------------------------ |
| `numpy`      | Core numerical computation, array operations | Multiple modules                                 |
| `scipy`      | ODE solver (`solve_ivp`)                     | `simulator`, `utils`, `visualiser`               |
| `matplotlib` | Static plotting                              | `visualiser`                                     |
| `tqdm`       | Progress bar for loops                       | `visualiser`, `simulator`                        |
| `sympy`      | Symbolic matrix construction (Jacobians)     | `utils/analytical_F.py`, `utils/analytical_H.py` |

Note: built-in standard libraries such as `os`, `sys`, `datetime`, `time`, `math`, and `json` are also used across multiple modules, but do not require separate installation.