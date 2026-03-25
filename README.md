## Vehicle State Estimation (EKF) — Repository

Comprehensive tools and examples for vehicle state estimation using Extended Kalman Filter (EKF), data processing, and visualization.

**What this repo contains**
- **Purpose**: Provide C++ EKF implementations, vehicle models, test data and Python data-processing/visualization notebooks to evaluate and develop state estimation algorithms.
- **Primary languages**: C++ (EKF and vehicle model) and Python (data processing / visualization).

**Quick Links**
- **Source & tests**: [src](src)
- **C++ includes / headers**: [include](include)
- **Python scripts & notebooks**: [py_scripts/data_processing](py_scripts/data_processing)
- **Raw / example data**: [data](data)
- **EKF implementation (C++)**: [kalman_filters](kalman_filters)

**Repository layout**
- **include/**: C++ header files for vehicle models and EKF interfaces (e.g., [include/kinematic_bicycle.h](include/kinematic_bicycle.h)).
- **src/**: Tests and example programs (see `src/test_ekf.cpp`).
- **kalman_filters/**: EKF implementation and helper files (`ekf.cpp`, `ekf.h`, `jacobiankinematic.h`).
- **py_scripts/data_processing/**: Jupyter notebooks and Python helpers for data visualization and bias estimation (see [py_scripts/data_processing/data_visualization.ipynb](py_scripts/data_processing/data_visualization.ipynb)).
- **data/**: Example CSV files and small test programs — contains measurement series and utilities.

**Getting started — Build (C++)**
- Prerequisites: a C++ compiler (Clang/GCC), Eigen (for linear algebra), and standard build tools.
- Example single-file build (used by the VS Code task):

```bash
/usr/bin/clang++ -g src/test_ekf.cpp -o bin/test_ekf \
	-I/opt/anaconda3/envs/kalman2/include/eigen3 \
	-I/opt/anaconda3/envs/kalman2/include/python3.11 \
	-Iinclude
```

- Or run the provided VS Code build task: open the file and use the `C/C++: clang++ build active file with include` task.

**Run examples & tests**
- Build the tests as above, then run the produced binary from `src/` or `bin/`.
- Example:

```bash
./src/test/test_ekf    # or ./bin/test_ekf if you built to bin/
```

**Python environment — data processing & visualization**
- Recommended packages: `pandas`, `numpy`, `matplotlib`, `jupyter`.
- Quick setup with pip:

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt  # create this file with pandas,numpy,matplotlib
```

- To open the visualization notebook:

```bash
jupyter lab py_scripts/data_processing/data_visualization.ipynb
```

**Data & Notebooks**
- Use the notebooks in [py_scripts/data_processing](py_scripts/data_processing) to visualize signals, estimate biases, and evaluate EKF performance. The notebooks include examples for:
	- Bias estimation (see `BIAS_ESTIMATION_METHODS.md`)
	- EKF feasibility report and tuning (see `EKF_FEASIBILITY_REPORT.md` and `EKF_IMPLEMENTATION_PARAMETERS.md`)

**Git / .gitignore notes**
- This repository intentionally ignores local artifacts such as `*.txt`, `*.dSYM/`, build outputs and interim data files. If you need to stop tracking already committed files, run:

```bash
# keep local files, stop tracking in git index
git ls-files -z -- '*.txt' | xargs -0 -r git rm --cached --ignore-unmatch
git ls-files -z | tr '\0' '\n' | grep '\.dSYM' | tr '\n' '\0' | xargs -0 -r git rm --cached --ignore-unmatch
git add .gitignore
git commit -m "Stop tracking .txt and .dSYM files"
git push origin main
```

**Recommended workflow**
- Use the Python notebooks for data exploration and to determine process/measurement noise characteristics.
- Implement or tune the EKF in `kalman_filters/` and test using `src/` test harnesses.
- Iterate: use notebooks to visualize results and compare EKF estimate against GPS traces.

**Contributing**
- Feel free to open issues or submit pull requests. When contributing:
	- Add unit tests or a small reproducible example for changes in C++ code.
	- Update notebooks or add a small script demonstrating new analysis in `py_scripts/data_processing`.

**License**
- This project is licensed under the MIT License — see [LICENSE](LICENSE) for details.

**Contact / Maintainers**
- For questions about the EKF implementation or datasets, open an issue in this repository.


