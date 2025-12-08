import streamlit as st
import plotly.graph_objects as go
import numpy as np
import random
import io
import time
from environment import Airspace
from planner import RoutePlanner

# --- Page Configuration ---
st.set_page_config(page_title="Intelligent Trajectory Simulator", layout="wide")
st.title("🚁 3D Trajectory Optimizer for UAS")

# --- Compatibility helper for rerun across Streamlit versions ---
def safe_rerun():
    """Call the appropriate Streamlit rerun API for the installed version.
    If neither is available, set a query param and ask the user to refresh."""
    # Preferred: new API
    if hasattr(st, "rerun"):
        try:
            st.rerun()
            return
        except Exception:
            pass

    # Fallback: older experimental name
    if hasattr(st, "experimental_rerun"):
        try:
            st.experimental_rerun()
            return
        except Exception:
            pass

    # Last resort: change a query param as a soft hint and ask the user
    try:
        st.experimental_set_query_params(_r=int(time.time()))
        st.info("Please refresh the page to complete the action.")
    except Exception:
        st.info("Please refresh the page to continue.")

# --- Initialize Session State ---
if 'custom_obstacles' not in st.session_state:
    st.session_state.custom_obstacles = []
if 'last_path' not in st.session_state:
    st.session_state.last_path = None

# --- Sidebar: Global Settings ---
with st.sidebar:
    st.header("⚙️ Simulation Parameters")
    st.info("Define the map size and physics rules.")
    grid_size = st.slider("Map Size (meters)", 20, 50, 30)

    # --- Weather Section ---
    st.subheader("🌬️ Weather Conditions")
    wind_alt = st.slider("Wind Start Altitude", 5, 20, 10, help="Wind only exists above this height.")
    wind_speed = st.slider("Wind Speed (m/s)", 0.0, 10.0, 5.0)
    wind_dir = st.slider("Wind Direction (°)", 0, 360, 180, help="0=East, 90=North, 180=West")

    direction_text = "➡️ East (+X)"
    if 45 < wind_dir <= 135:
        direction_text = "⬆️ North (+Y)"
    elif 135 < wind_dir <= 225:
        direction_text = "⬅️ West (-X)"
    elif 225 < wind_dir <= 315:
        direction_text = "⬇️ South (-Y)"
    st.caption(f"Wind Blowing: {direction_text}")

    # --- Physics Section ---
    st.subheader("🔋 Physics Costs")
    climb_cost = st.slider("Climb Cost", 1.0, 5.0, 2.0, help="Energy cost to move UP")
    safety_penalty = st.slider("Low Altitude Penalty", 0.0, 20.0, 10.0, help="Penalty for flying too close to ground")

    # --- Mission Section ---
    st.subheader("📍 Mission Waypoints")
    c1, c2 = st.columns(2)
    with c1:
        start_x = st.number_input("Start X", 0, grid_size-1, 2)
        start_y = st.number_input("Start Y", 0, grid_size-1, 2)
    with c2:
        goal_x = st.number_input("Goal X", 0, grid_size-1, grid_size-2)
        goal_y = st.number_input("Goal Y", 0, grid_size-1, grid_size-2)

# --- Helper: serialize obstacles for caching ---
def serialize_obstacles(obstacles):
    """Convert list-of-dicts into a tuple-of-tuples (hashable) in stable order."""
    serialized = []
    for o in obstacles:
        # ensure consistent key order
        serialized.append((o.get('x', 0), o.get('y', 0), o.get('dx', 0), o.get('dy', 0), o.get('dz', 0)))
    return tuple(serialized)

def deserialize_obstacles(serialized):
    """Convert serialized obstacles back to list of dicts."""
    return [{'x': t[0], 'y': t[1], 'dx': t[2], 'dy': t[3], 'dz': t[4]} for t in serialized]

# --- Helper: create sample scenarios ---
def load_sample(name):
    st.session_state.custom_obstacles = []
    if name == 'City (dense)':
        for i in range(5, min(grid_size, 25), 4):
            for j in range(5, min(grid_size, 25), 4):
                st.session_state.custom_obstacles.append({'x': i, 'y': j, 'dx': 3, 'dy': 3, 'dz': random.randint(6, 14)})
    elif name == 'Sparse Field':
        for _ in range(8):
            rx = random.randint(0, max(0, grid_size-5))
            ry = random.randint(0, max(0, grid_size-5))
            st.session_state.custom_obstacles.append({'x': rx, 'y': ry, 'dx': random.randint(2,4), 'dy': random.randint(2,4), 'dz': random.randint(5,12)})

# --- Main Layout ---
col_viz, col_builder = st.columns([3, 1])

# --- RIGHT COLUMN: Obstacle Tools ---
with col_builder:
    st.subheader("🏗️ Obstacle Manager")
    tab1, tab2 = st.tabs(["Manual Add", "Random Gen"])

    with tab1:
        with st.form("manual_obs"):
            st.markdown("**Add Specific Building**")
            ox = st.number_input("X Pos", 0, grid_size, 10)
            oy = st.number_input("Y Pos", 0, grid_size, 10)
            w = st.number_input("Width (X)", 1, 10, 4)
            l = st.number_input("Length (Y)", 1, 10, 4)
            h = st.number_input("Height (Z)", 1, 15, 10)

            if st.form_submit_button("➕ Add Single"):
                st.session_state.custom_obstacles.append({'x': int(ox), 'y': int(oy), 'dx': int(w), 'dy': int(l), 'dz': int(h)})
                st.success("Building Added!")

    with tab2:
        with st.form("random_obs"):
            st.markdown("**Generate Random Field**")
            num_rand = st.number_input("Count", 1, 50, 5)
            max_h = st.number_input("Max Height", 5, 20, 12)

            if st.form_submit_button("🎲 Generate"):
                for _ in range(int(num_rand)):
                    rx = random.randint(0, max(0, grid_size-5))
                    ry = random.randint(0, max(0, grid_size-5))
                    rw = random.randint(2, 5)
                    rl = random.randint(2, 5)
                    rh = random.randint(5, int(max_h))
                    st.session_state.custom_obstacles.append({'x': rx, 'y': ry, 'dx': rw, 'dy': rl, 'dz': rh})
                st.success(f"Added {num_rand} buildings.")

    st.divider()
    # Sample scenarios
    sample = st.selectbox("Load sample scenario", ["(none)", "City (dense)", "Sparse Field"])
    if st.button("Load Sample") and sample != "(none)":
        load_sample(sample)
        safe_rerun()

    # Manage Existing Obstacles
    if st.session_state.custom_obstacles:
        st.write(f"**Total Buildings:** {len(st.session_state.custom_obstacles)}")
        if st.button("🗑️ Clear Map", type="secondary"):
            st.session_state.custom_obstacles = []
            safe_rerun()

# --- Simulation core wrapped and cached ---
@st.cache_data
def run_simulation_cached(grid_size, wind_alt, wind_speed, wind_dir, climb_cost, safety_penalty, obstacles_serialized, start_node, goal_node):
    # reconstruct obstacles
    obstacles = deserialize_obstacles(obstacles_serialized)
    env = Airspace(size=(grid_size, grid_size, 20))
    env.set_wind(z_level=wind_alt, intensity=wind_speed, direction_degrees=wind_dir)

    # Place obstacles into grid
    for obs in obstacles:
        x, y, dx, dy, h = int(obs['x']), int(obs['y']), int(obs['dx']), int(obs['dy']), int(obs['dz'])
        if x + dx <= grid_size and y + dy <= grid_size:
            env.grid[x:x+dx, y:y+dy, 0:h] = 1

    planner = RoutePlanner(env, {
        'climb_cost': climb_cost,
        'wind_penalty': wind_speed,
        'safety_penalty': safety_penalty
    })

    path = planner.find_path(start_node, goal_node)
    return env, path

# --- LEFT COLUMN: Simulation & Visualization ---
with col_viz:
    if st.button("🚀 Run Simulation", type="primary"):
        with st.spinner("Calculating optimal path..."):
            try:
                start_node = (int(start_x), int(start_y), 0)
                goal_node = (int(goal_x), int(goal_y), 5)

                obstacles_serialized = serialize_obstacles(st.session_state.custom_obstacles)
                env, path = run_simulation_cached(
                    int(grid_size),
                    int(wind_alt),
                    float(wind_speed),
                    int(wind_dir),
                    float(climb_cost),
                    float(safety_penalty),
                    obstacles_serialized,
                    start_node,
                    goal_node
                )

                # Visualization using Plotly 3D
                fig = go.Figure()

                obs_x, obs_y, obs_z = np.where(env.grid == 1)
                fig.add_trace(go.Scatter3d(
                    x=obs_x, y=obs_y, z=obs_z,
                    mode='markers',
                    marker=dict(symbol='square', size=5, opacity=0.4),
                    name='Buildings'
                ))

                if path:
                    px, py, pz = zip(*path)
                    fig.add_trace(go.Scatter3d(
                        x=px, y=py, z=pz,
                        mode='lines+markers',
                        line=dict(color='red', width=6),
                        marker=dict(size=4, color='yellow'),
                        name='Optimal Path'
                    ))
                    st.success(f"Path found with {len(path)} nodes")
                    st.session_state.last_path = path
                else:
                    st.error("❌ No path found! The destination is unreachable or blocked.")
                    st.session_state.last_path = None

                # DRAW START & GOAL
                fig.add_trace(go.Scatter3d(
                    x=[int(start_x)], y=[int(start_y)], z=[0],
                    mode='markers', marker=dict(size=10, color='green'), name='Start'
                ))
                fig.add_trace(go.Scatter3d(
                    x=[int(goal_x)], y=[int(goal_y)], z=[5],
                    mode='markers', marker=dict(size=10, color='blue'), name='Goal'
                ))

                fig.update_layout(
                    scene=dict(
                        aspectmode='data',
                        zaxis=dict(range=[0, 20], title="Altitude (m)"),
                        xaxis=dict(title="X (m)"),
                        yaxis=dict(title="Y (m)"),
                    ),
                    margin=dict(l=0, r=0, b=0, t=0),
                    height=600
                )

                st.plotly_chart(fig, use_container_width=True)

            except Exception as e:
                st.exception(e)
    else:
        st.info("👈 Add buildings on the right or configure settings on the left, then click 'Run Simulation'.")

    # Post-simulation actions: download, inspect
    st.divider()
    st.subheader("📥 Output & Export")
    if st.session_state.last_path:
        # Create CSV text
        csv_lines = ["x,y,z"]
        csv_lines += [f"{p[0]},{p[1]},{p[2]}" for p in st.session_state.last_path]
        csv_text = "\n".join(csv_lines)

        st.download_button("Download Path CSV", data=csv_text, file_name="trajectory.csv", mime="text/csv")

        if st.button("Show Path Points"):
            st.write(st.session_state.last_path)
    else:
        st.caption("Run a simulation to enable download and inspection of the resulting path.")
