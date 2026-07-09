# 🧠 AUE8088 PA1 — Neural Networks from Scratch

> Course: **AUE8088 — Understanding & Application of Deep Learning**, Hanyang University (Graduate), 2026
> Study project @ **Automotive Intelligence Laboratory**

Implement the **forward propagation and backpropagation** of a fully-connected neural network **from scratch using only Python + NumPy** — no PyTorch / TensorFlow / autodiff. The task is **driving-behavior classification** from vehicle IMU sensor data.

## 📌 Assignment Progression

| Part | Notebook | Description |
| ---- | -------- | ----------- |
| **PA1-1** | `PA1-1_NN_from_scratch.ipynb` | Toy dataset (synthetic vehicle dynamics). 1-hidden-layer net `4 → 16 → 3` with ReLU + Softmax; forward/backprop + gradient descent from scratch. Classes: straight / brake / turn. |
| **PA1-2** | `PA1-2_NN_from_scratch_RealData.ipynb` | Real sensor data (`sensor_raw.csv`, 6 features: AccX/Y/Z, GyroX/Y/Z). 4 classes: acceleration / right turn / left turn / braking. Standardization, 2000 epochs, PCA visualization. |
| **PA1-3** | `PA1-3_NN_from_scratch_RealData_Module.ipynb` | Refactor into modular OOP layers: `Linear`, `ReLU`, `SoftmaxWithLoss`, `TwoLayerNet` — each with its own `forward` / `backward`. |

Rendered results are also saved as `.html` alongside each notebook.

## 🔑 Concepts Implemented

- Forward propagation (affine → ReLU → affine → Softmax)
- Backpropagation via analytic gradients (no autodiff)
- Cross-entropy loss, gradient descent
- Vectorized NumPy matrix ops (minimize `for`-loops via broadcasting)
- Modular layer abstraction (PyTorch-like `Linear`/`ReLU` modules by hand)

## 🛠️ Stack

- **Python 3**, **NumPy** (core), **Pandas** (data), **Matplotlib** (curves)
- **scikit-learn** — PCA / t-SNE for visualization only

## 📂 Dataset

`driving-behavior-dataset/` — vehicle accelerometer + gyroscope readings (~3,600 samples, 6 features, 4 classes) with several feature-engineering variants.
