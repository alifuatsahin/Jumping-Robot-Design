# 🦘 Jumping Robot Design and Optimization  
This project focuses on the **data-driven design, simulation, and optimization of an energy-efficient jumping robot**. The goal is to maximize jumping distance while minimizing energy consumption through simulation-based optimization and real-world prototyping.  
## 📌 Project Overview  
We designed and developed a configurable jumping robot in **PyBullet**, analyzed key mechanical parameters, and used **genetic and Bayesian optimization** to improve its performance. A physical prototype was built and tested to validate the simulation results.  
### 👨‍🔬 Authors  
- Ali Fuat Şahin ([alifuat.sahin@epfl.ch](mailto:alifuat.sahin@epfl.ch))  
- Jade Therras  
- Davide Lisi  
- Antonio Ruiz Senac  
## ⚙️ Features  
- **Simulation environment** built in PyBullet with reconfigurable parameters  
- **Parameter exploration**: link lengths, spring stiffness, compression ratios  
- **Optimization algorithms**:  
  - Genetic Algorithm using PyGAD  
  - Bayesian Optimization using Scikit-learn GPR  
- **Fabricated and tested** two real prototypes: *Kenan 1.0* and *Kenan 2.0*  
- Performance metrics: Jump height, distance, energy consumption  
## 🧪 Simulation & Optimization Details  
### Parameters Optimized  
- Link lengths and coefficients  
- Spring stiffness  
- Rest angle and compression  
- Ground link angle  
### Objective Function  
Maximize jump distance \( D \) while minimizing robot size and energy:  
fitness = D / L_max
energy = 0.5 * k * delta_compression^2
### Optimization Strategy  
- **Genetic Algorithm**:  
  - 5000 generations × 10 individuals  
- **Bayesian Optimization**:  
  - Local fine-tuning via Gaussian Process Regression  
## 📊 Results Summary  
| Robot           | Jump Height | Jump Distance | Energy Used |  
|-----------------|-------------|----------------|--------------|  
| Kenan 1.0 (real) | 15 cm       | 25 cm          | ~1.5 J       |  
| Simulated Optimum | 38.9 cm    | 90.5 cm        | 0.72 J       |  
| Kenan 2.0 (real) | 12 cm       | 35 cm          | ~1.3 J       |  
*→ Jump distance improved by ~40% in real hardware with optimized geometry.*  
## 🏗️ Repository Structure  
├── simulation/ # PyBullet simulation code
├── optimization/ # Genetic & Bayesian optimization scripts
├── prototype_design/ # CAD and fabrication notes
├── report/ # Final report and documentation
├── plots/ # Performance visualizations
└── Solution.txt # Best parameter set from optimization
## 🧰 Requirements  
- Python 3.8+  
- [PyBullet](https://github.com/bulletphysics/bullet3)  
- [PyGAD](https://pygad.readthedocs.io/)  
- [Scikit-learn](https://scikit-learn.org/)  
- NumPy, Matplotlib, Pandas  
Install dependencies with:  
```bash
pip install -r requirements.txt
```
##📄 Project Report
For full details, please refer to the 📘 final report (PDF).
##📬 Contact
For questions or collaboration, contact:

Ali Fuat Şahin – alifuat.sahin@epfl.ch

