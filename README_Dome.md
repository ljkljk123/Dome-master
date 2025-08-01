# Dome: Fast and Robust LiDAR Place Recognition via Multi-View Feature Fusion

Dome is a lightweight, real-time LiDAR place recognition algorithm based on spherical multi-view projections and descriptor fusion. It achieves high recognition accuracy and robustness in complex environments, while maintaining over **250Hz** inference speed without requiring deep learning or GPU acceleration.

---

## 🔍 Overview

Dome projects LiDAR point clouds onto three spherical views (R-P, P-Y, R-Y), and extracts geometric features such as relative depth, density, and elevation. It enhances feature discriminability through density-aware convolution (P-Y view) and geometric-aware weighting (R-Y view). Fast cosine similarity in the frequency domain is used for loop detection, accelerated by FFTW.

---

## 🚀 Features

- 🔁 Robust loop closure under occlusion and rotation
- ⚡ High-speed descriptor matching (250Hz+)
- 📦 Pure C++ implementation with FFTW & OpenMP
- 🔎 No dependency on deep learning or GPU
- 📊 Strong performance on KITTI, MulRan, and KAIST datasets

---

## 📦 Dependencies

- C++17
- PCL ≥ 1.9
- FFTW3
- OpenCV ≥ 3.4
- OpenMP (optional but recommended)

---

## 🧭 Dataset Support

- KITTI
- MulRan
- KAIST Urban

---

## 🛠️ Build Instructions

```bash
git clone https://github.com/YourName/Dome.git
cd Dome
mkdir build && cd build
cmake ..
make -j8
```

---

## 🧪 Run Example

```bash
./dome_loop_detector \
    --pc_folder ./data/kitti_05/velodyne \
    --pose_file ./data/kitti_05/poses.txt
```

---

## 📈 Performance

| Dataset   | Recall@1 | Time per Frame |
|-----------|----------|----------------|
| KITTI 05  | 98.2%    | 0.0041s        |
| MulRan    | 95.3%    | 0.0043s        |
| KAIST     | 96.7%    | 0.0040s        |

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