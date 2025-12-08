# 🚁 Intelligent UAS Trajectory Optimizer  
[![Python Version](https://img.shields.io/badge/Python-3.8%2B-blue.svg)](https://www.python.org/)
[![Streamlit](https://img.shields.io/badge/Frontend-Streamlit-FF4B4B.svg)](https://streamlit.io/)
[![Plotly](https://img.shields.io/badge/Visualization-Plotly-3DA639.svg)](https://plotly.com/)
[![Live Demo](https://img.shields.io/badge/Live%20Demo-Streamlit-brightgreen?logo=streamlit)](https://intelligent-trajectory-simulator-agm5vvdzdxarqosjusmmv8.streamlit.app/)

---

## 🌐 Live Application  
👉 **Run the simulator instantly:**  
### **https://intelligent-trajectory-simulator-agm5vvdzdxarqosjusmmv8.streamlit.app/**  

No installation required. Build environments, configure wind, and visualize 3D UAS trajectories in real time.

---

## 📌 Overview  

The **Intelligent UAS Trajectory Optimizer** is a real-time simulation and path-planning engine for **Unmanned Aerial Systems (UAS)** operating in complex urban environments.  

It computes **energy-optimal 3D trajectories** using a physics-aware **A\*** search algorithm that models:

- Wind vectors  
- 26-direction 3D movement  
- Gravity-aware climb & descent energy  
- Voxelized buildings  
- Low-altitude safety penalties  
- Segment-wise energy diagnostics  
- 3D wind cone visualization  

---

## 🧠 Key Features

### ✈️ Physics-Based Trajectory Optimization
- Gravity + climb/descent energy cost  
- Wind projection onto movement vector  
- Low-altitude safety penalty  

### 🏙️ Urban Obstacle Environment
- Manual or random obstacle creation  
- Voxel grid (20 altitude layers)  
- Real-time obstacle visualization  

### 📊 Visualization
- Plotly 3D renderer  
- Wind cones (adjustable density/sizeref)  
- Path markers + hover tooltips  
- CSV export  

### ⚡ Interactive Streamlit UI
- Configurable mission parameters  
- Wind control panel  
- On-the-fly recomputation  

---

## 🛠️ Tech Stack

```
Language:     Python 3.x
Frontend:     Streamlit
Visualization: Plotly 3D
Math/Arrays:  NumPy
Algorithm:    A* Search (26-direction 3D)
Data Handling: Session State, CSV Export
```

---

## 🏗️ System Architecture (ASCII Diagram)

```
+--------------------------------------------------------------------------------+
|                                Streamlit UI                                    |
|  - User inputs (start, goal)                                                   |
|  - Obstacle builder                                                            |
|  - Wind settings                                                               |
+--------------------------------------+-----------------------------------------+
                                       |
                                       v
+--------------------------------------+-----------------------------------------+
|                             Environment Module                                |
|  - 3D occupancy grid                                                        |
|  - Wind vector field                                                        |
|  - Obstacle mapping                                                         |
+--------------------------------------+-----------------------------------------+
                                       |
                                       v
+--------------------------------------+-----------------------------------------+
|                                Route Planner                                   |
|  - 26-direction A* search                                                     |
|  - Energy cost model (climb/descend/wind/safety)                              |
|  - Returns: path, runtime, cost, nodes explored, breakdown                     |
+--------------------------------------+-----------------------------------------+
                                       |
                                       v
+--------------------------------------------------------------------------------+
|                            Plotly 3D Renderer                                  |
|  - Renders obstacles                                                           |
|  - Renders optimal trajectory                                                  |
|  - Renders wind cones                                                          |
|  - Hover energy diagnostics                                                    |
+--------------------------------------------------------------------------------+
```

---

## 📂 Project Structure

```
├── app.py              # Streamlit UI + Visualization
├── planner.py          # A* trajectory planner (energy-aware)
├── environment.py      # Grid, wind field, obstacle logic
├── requirements.txt
├── images/
│   ├── demo_setup.png
│   └── demo_path.png
└── README.md
```

---

## 📸 Demo Screenshots

### 🧭 Simulation Setup  
![Simulation Setup](images/demo_setup.png)

### 🚀 3D Trajectory Visualization  
![3D Trajectory Visualization](images/demo_path.png)

---

## 🛠️ Installation (Local)

```bash
git clone https://github.com/yourusername/uas-trajectory-optimizer.git
cd uas-trajectory-optimizer
```

### 1. Virtual Environment  
#### Windows
```bash
python -m venv venv
venv\Scripts\activate
```
#### macOS / Linux
```bash
python3 -m venv venv
source venv/bin/activate
```

### 2. Install Requirements
```bash
pip install -r requirements.txt
```

### 3. Run App
```bash
streamlit run app.py
```

Open browser:  
```
http://localhost:8501
```

---

## 🔍 Technical Deep Dive

### Energy Model

```
energy =
    horizontal_distance
  + climb/descent cost
  + wind_resistance_projection
  + low_altitude_penalty
```

### Route Planning (planner.py)

- A* with full 26-direction neighbor expansion  
- Euclidean heuristic  
- Physics-informed edge cost  
- Outputs:
  - path  
  - runtime  
  - total cost  
  - nodes explored  
  - energy breakdown  

### Visualization (app.py)
- Obstacles  
- Path  
- Wind cones  
- Hover labels  
- CSV export  

---

## 🔮 Future Enhancements
- B-spline smoothing  
- Battery model integration  
- Dynamic obstacle avoidance  
- Real-world weather ingestion  
- MAVLink waypoint export  

---

## 🤝 Contributing
Contributions and feature ideas are welcome!
