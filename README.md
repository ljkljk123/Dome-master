# Dome: Fast and Robust LiDAR Place Recognition via Spherical Three-View Feature Fusion

Dome is a lightweight, real-time LiDAR place recognition algorithm based on spherical multi-view projections and descriptor fusion. It achieves high recognition accuracy and robustness in complex environments, while maintaining very high processing speed without relying on deep learning or GPU acceleration.

---

## 🔍 Overview

Dome projects LiDAR point clouds onto three spherical views (R-P, P-Y, R-Y), and extracts geometric features such as relative depth, density, and elevation. It enhances feature discriminability through density-aware convolution (P-Y view) and geometric-aware weighting (R-Y view). Fast cosine similarity in the frequency domain is used for loop detection, accelerated by FFTW.

---

## 🚀 Features

- 🔁 Robust loop closure under occlusion and rotation
- ⚡ High-speed descriptor matching (250Hz+)
- 📦 Pure C++ implementation with FFTW
- 🔎 No dependency on deep learning or GPU
- 📊 Strong performance on KITTI, and MulRan datasets

---

## 📦 Environment & Dependencies

**Tested Environment**
- Ubuntu 18.04
- **ROS Melodic**
- CMake ≥ 3.10
- CPU-only (no GPU required)

**Core Dependencies**
- **C++17**
- PCL = 1.8.1  (default in ROS Melodic)
- OpenCV ≥ 3.2  (tested with 3.2.0)
- **FFTW3 = 3.3.7**
- Eigen3 = 3.3.4
- ROS packages: `roscpp`, `pcl_ros`, `cv_bridge` (depending on usage)

## 🛠️ Build Instructions

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
https://github.com/ljkljk123/Dome-master.git
```

---

## 🧪 Run Example

```bash

```

---

## 📈 Performance

---

## 📄 Citation

```bibtex
@article{YourPaper2025,
  title={Dome: Fast and Robust LiDAR Place Recognition via Multi-View Feature Fusion},
  author={Your Name et al.},
  journal={TBD},
  year={2025}
}
```

---

## 📬 Contact

For questions or feedback, feel free to open an issue or contact us at [youremail@domain.com].
