# 🎯 Optimal State Estimation Practice

> Hands-on study of optimal state estimation, implemented **from scratch in pure NumPy** — no black-box filtering libraries.

Part of my study at the **Automotive Intelligence Laboratory (Hanyang University)**. The notebooks build up from discrete-time simulation and least squares, through the Kalman Filter, to nonlinear estimation with the Extended Kalman Filter and Particle Filter.

## 📌 Topics Covered

| Notebook | Topic | Key Implementation |
| -------- | ----- | ------------------ |
| `practice01_Simulation.ipynb` | Discrete-time state simulation | Free-fall dynamics via state-transition matrix `F` (= KF *prediction* step) |
| `practice02_Least Square Estimateion.ipynb` | Least squares estimation | Batch LS `x̂ = (HᵀH)⁻¹Hᵀy` and Recursive Least Squares (RLS) |
| `practice03_State_Cov_Propagation.ipynb` | State & covariance propagation | Mean/covariance propagation `P = F·P·Fᵀ + Q`, steady-state analysis |
| `practice04_The_discrete_time_Kalman_Filter.ipynb` | Discrete-time Kalman Filter | Predict/update cycle, Kalman gain, Q/R/P₀ tuning study (Cases 1–5), RMSE comparison |
| `practice05_2d_Kalman_filter.ipynb` | 2D Kalman Filter | Constant-Velocity (CV) model, 4-state `[x, y, vₓ, v_y]` tracking |
| `practice06_Extended_Kalman_filter.ipynb` | Extended Kalman Filter | Nonlinear bearing measurement, Jacobian linearization, CV vs. Dead-Reckoning models |
| `practice07_particle_filter.ipynb` | Particle Filter | Predict → weight → resample → estimate; comparison of EKF-CV, EKF-DR, PF-DR |

`Practice_07_Particle_filter_solution.ipynb` contains the full, commented particle-filter benchmark.

## 🧠 EKF vs. PF — core difference

| | EKF | PF |
| --- | --- | --- |
| State representation | mean + covariance | N particles |
| Nonlinearity | Jacobian (1st-order) | sampling |
| Gaussian assumption | required | not required |
| Multi-modal distribution | not handled | handled |

## 🛠️ Stack

- **Python 3**, **NumPy** (all algorithms coded from first principles)
- **Matplotlib** (trajectory / covariance visualization)
- **SciPy** / **h5py** (`.mat` data I/O)

## 📊 Results

Selected output figures are in [`outputs/`](./outputs):
- `KF_best_result.png`, `Q_tuning_result.png` — Kalman Filter tuning study
- `CA_KF_best_result.png`, `CA_Q_tuning_result.png`, `CA_P_covariance.png`, `CA_accel_result.png` — Constant-Acceleration model

## 📂 Data

- `data.mat` — free-fall simulation with noisy altitude measurements
- `measurement_simulation.mat` — constant-velocity ground truth + noisy position measurements
