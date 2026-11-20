# Adaptive Quadcopter Control

A MATLAB-based simulation study exploring online mass estimation and adaptive altitude control under payload disturbances.

## Overview

This project investigates how a quadcopter can maintain altitude when its physical parameters change during flight.

An online Recursive Least Squares (RLS) estimator identifies changes in vehicle mass, allowing the altitude controller to adapt its control response without requiring prior knowledge of the new payload.

## System Configuration

| Parameter | Value |
|---|---:|
| Initial vehicle mass | 1.20 kg |
| Payload mass | 0.24 kg |
| Final vehicle mass | 1.44 kg |
| Payload change time | 15 s |
| Simulation duration | 30 s |
| Integration time step | 0.005 s |
| Reference altitude | 5 m |
| Baseline acceleration noise | 0.15 m/s² |

## Control Architecture

The simulation evaluates two configurations:

- **Fixed-mass control:** Uses the original vehicle mass throughout the simulation.
- **Adaptive control:** Uses online RLS mass estimation to adjust the controller's mass-dependent response.

Both configurations operate under identical simulation conditions to enable a controlled performance comparison.

## Experimental Results

### 1. Adaptive Mass Estimation

The estimator successfully identifies the payload-induced mass change.

| Metric | Result |
|---|---:|
| Actual final mass | 1.4400 kg |
| Estimated final mass | 1.4414 kg |
| Final estimation error | 0.097% |
| Settling time | 2.660 s |

### 2. Adaptive Control Performance

Post-payload performance comparison:

| Metric | Fixed Control | Adaptive Control |
|---|---:|---:|
| Tracking RMSE | 0.316272 m | 0.074058 m |
| Maximum altitude error | 0.617314 m | 0.189609 m |
| Recovery time | 10.035 s | 2.440 s |
| RMS thrust | 14.131821 N | 14.130713 N |
| Thrust-rate RMS | 0.808284 N/s | 1.035963 N/s |
| Peak thrust rate | 6.292593 N/s | 8.326807 N/s |

The adaptive controller reduced post-payload tracking error and recovery time while maintaining essentially unchanged RMS thrust demand.

This improvement came with increased thrust variation and peak thrust-rate demand.

### 3. Sensor-Noise Sensitivity

Adaptive controller post-payload tracking performance under different acceleration measurement noise levels:

| Noise (m/s²) | Tracking RMSE (m) |
|---:|---:|
| 0.00 | 0.073990 |
| 0.15 | 0.074058 |
| 0.50 | 0.074341 |
| 1.00 | 0.075041 |

The fixed controller maintained an RMSE of 0.316272 m across these tested conditions.

The adaptive controller retained a tracking improvement of approximately 76% across all four noise levels.

## Project Status

**Completed**
- Quadcopter dynamics simulation
- Online RLS mass estimation
- Fixed versus adaptive control comparison
- Control effort and smoothness evaluation
- Initial sensor-noise sensitivity experiments

**Next**
- Multi-seed statistical validation
- Further robustness and sensitivity testing
- Extended disturbance scenarios

## Limitations

Results currently represent MATLAB simulation experiments, not physical flight tests. Noise sensitivity results use a single random seed per condition. Further statistical validation is required before drawing broader robustness conclusions.

## Saved Experimental Data

- `phase02_results.mat` — Adaptive control performance results.
- `phase03_noise_results.mat` — Sensor-noise sensitivity results.

---

*An ongoing model-based investigation into adaptive control, online parameter estimation, and autonomous aerial systems.*