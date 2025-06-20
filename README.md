# 3D Path-Following Guidance for Fixed-Wing UAVs

This repository contains MATLAB code to reproduce the simulation results and figures from the paper:

E. M. Coates, T. Hamel and T. I. Fossen, "Almost Global Three-Dimensional Path-Following Guidance Law for Arbitrary Curved Paths," 2023 62nd IEEE Conference on Decision and Control (CDC), Singapore, Singapore, 2023, pp. 2630-2636, doi: 10.1109/CDC49753.2023.10384138

You can access the paper [here](https://doi.org/10.1109/CDC49753.2023.10384138) or [here](https://ntnuopen.ntnu.no/ntnu-xmlui/bitstream/handle/11250/3114896/CDC_2023%2B%252811%2529.pdf?sequence=1).

---

## 🔍 Overview

The code implements a 3D path-following guidance law tailored for fixed-wing UAVs flying along arbitrary spatial curves. The control scheme is designed using a cascaded structure with:

- **Outer loop**: Generates a desired inertial velocity direction to reduce cross-track error.
- **Inner loop**: Tracks the desired air-relative velocity direction using normal acceleration control.
- **Wind compensation**: Wind triangle resolution ensures robustness in windy conditions.

---

## ▶️ Getting Started

### Requirements

- MATLAB R2016b or newer
- No additional toolboxes required

### Run the Simulation

1. Clone the repository:
   ```bash
   git clone https://github.com/erlendcoates/3d-path-following-CDC2023.git
   cd 3d-path-following-CDC2023
   ```

2. Open `run3DGuidanceSim.m` in MATLAB.

3. Run the script:
   ```matlab
   run3DGuidanceSim
   ```

4. EPS figures will be saved in the `figures/` folder.

---

## 📄 Reference

If you use this code in your own work, please cite:

```
@inproceedings{coates2023pathfollowing,
  title     = {Almost Global Three-Dimensional Path-Following Guidance Law for Arbitrary Curved Paths},
  author    = {Coates, Erlend M. and Hamel, Tarek and Fossen, Thor I.},
  booktitle = {2023 62nd IEEE Conference on Decision and Control (CDC)},
  year      = {2023},
  pages={2630-2636},
  doi={10.1109/CDC49753.2023.10384138}
}
```

---
## ⚠️ Errata

In Fig.2 in the paper (the wind triangle), the direction of the wind velocity vector $$v_w$$ should be reversed.

---

## 📬 Contact

For questions or feedback, please contact [Erlend M. Coates](mailto:erlend.coates@ntnu.no).
