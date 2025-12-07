import streamlit as st
import plotly.graph_objects as go
import numpy as np
import random
from environment import Airspace
from planner import RoutePlanner

# --- Page Configuration ---
st.set_page_config(page_title="Intelligent Trajectory Simulator", layout="wide")
st.title("🚁 3D Trajectory Optimizer for UAS")

# --- Initialize Session State ---
# This "remembers" obstacles between button clicks
if 'custom_obstacles' not in st.session_state:
    st.session_state.custom_obstacles = []

# --- Sidebar: Global Settings ---
with st.sidebar:
    st.header("⚙️ Simulation Parameters")
    
    st.info("Define the map size and physics rules.")
    grid_size = st.slider("Map Size (meters)", 20, 50, 30)
    
    # --- Weather Section ---
    st.subheader("🌬️ Weather Conditions")
    wind_alt = st.slider("Wind Start Altitude", 5, 20, 10, help="Wind only exists above this height.")
    wind_speed = st.slider("Wind Speed (m/s)", 0.0, 10.0, 5.0)
    
    #Wind Direction Slider
    wind_dir = st.slider("Wind Direction (°)", 0, 360, 180, help="0=East, 90=North, 180=West")
    
    # Visual helper for wind direction
    direction_text = "➡️ East (+X)"
    if 45 < wind_dir <= 135: direction_text = "⬆️ North (+Y)"
    elif 135 < wind_dir <= 225: direction_text = "⬅️ West (-X)"
    elif 225 < wind_dir <= 315: direction_text = "⬇️ South (-Y)"
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

# --- Main Layout ---
col_viz, col_builder = st.columns([3, 1])

# --- RIGHT COLUMN: Obstacle Tools ---
with col_builder:
    st.subheader("🏗️ Obstacle Manager")
    
    # Tabs for Manual vs Random
    tab1, tab2 = st.tabs(["Manual Add", "Random Gen"])
    
    # TAB 1: Manual Placement
    with tab1:
        with st.form("manual_obs"):
            st.markdown("**Add Specific Building**")
            ox = st.number_input("X Pos", 0, grid_size, 10)
            oy = st.number_input("Y Pos", 0, grid_size, 10)
            w = st.number_input("Width (X)", 1, 10, 4)
            l = st.number_input("Length (Y)", 1, 10, 4)
            h = st.number_input("Height (Z)", 1, 15, 10)
            
            if st.form_submit_button("➕ Add Single"):
                st.session_state.custom_obstacles.append({'x': ox, 'y': oy, 'dx': w, 'dy': l, 'dz': h})
                st.success("Building Added!")

    # TAB 2: Random Generation
    with tab2:
        with st.form("random_obs"):
            st.markdown("**Generate Random Field**")
            num_rand = st.number_input("Count", 1, 50, 5)
            max_h = st.number_input("Max Height", 5, 20, 12)
            
            if st.form_submit_button("🎲 Generate"):
                for _ in range(num_rand):
                    # Random logic: ensure it fits in grid
                    rx = random.randint(0, grid_size-5)
                    ry = random.randint(0, grid_size-5)
                    rw = random.randint(2, 5)
                    rl = random.randint(2, 5)
                    rh = random.randint(5, max_h)
                    st.session_state.custom_obstacles.append({'x': rx, 'y': ry, 'dx': rw, 'dy': rl, 'dz': rh})
                st.success(f"Added {num_rand} buildings.")

    # Manage Existing Obstacles
    if st.session_state.custom_obstacles:
        st.divider()
        st.write(f"**Total Buildings:** {len(st.session_state.custom_obstacles)}")
        if st.button("🗑️ Clear Map", type="secondary"):
            st.session_state.custom_obstacles = []
            st.rerun()

# --- LEFT COLUMN: Simulation & Visualization ---
with col_viz:
    if st.button("🚀 Run Simulation", type="primary"):
        with st.spinner("Calculating optimal path..."):
            
            # 1. Initialize Environment
            env = Airspace(size=(grid_size, grid_size, 20))
            
            # 2. Set Wind with Direction
            env.set_wind(
                z_level=wind_alt, 
                intensity=wind_speed, 
                direction_degrees=wind_dir
            )
            
            # 3. Place User Obstacles
            for obs in st.session_state.custom_obstacles:
                x, y, dx, dy, h = obs['x'], obs['y'], obs['dx'], obs['dy'], obs['dz']
                # Safety check to prevent crashing if grid size was reduced
                if x+dx <= grid_size and y+dy <= grid_size:
                    env.grid[x:x+dx, y:y+dy, 0:h] = 1

            # 4. Initialize Planner and Find Path
            planner = RoutePlanner(env, {
                'climb_cost': climb_cost,
                'wind_penalty': wind_speed, 
                'safety_penalty': safety_penalty
            })
            
            start_node = (start_x, start_y, 0)
            goal_node = (goal_x, goal_y, 5)
            
            path = planner.find_path(start_node, goal_node)

            # 5. Visualization using Plotly 3D
            fig = go.Figure()

            # DRAW OBSTACLES 
            # Find all grid cells that are obstacles
            obs_x, obs_y, obs_z = np.where(env.grid == 1)
            fig.add_trace(go.Scatter3d(
                x=obs_x, y=obs_y, z=obs_z,
                mode='markers',
                marker=dict(symbol='square', size=5, color='gray', opacity=0.4),
                name='Buildings'
            ))

            # DRAW PATH
            if path:
                px, py, pz = zip(*path) 
                fig.add_trace(go.Scatter3d(
                    x=px, y=py, z=pz,
                    mode='lines+markers',
                    line=dict(color='red', width=6),
                    marker=dict(size=4, color='yellow'),
                    name='Optimal Path'
                ))
            else:
                st.error("❌ No path found! The destination is unreachable or blocked.")

            # DRAW START & GOAL
            fig.add_trace(go.Scatter3d(
                x=[start_x], y=[start_y], z=[0], 
                mode='markers', marker=dict(size=10, color='green'), name='Start'
            ))
            fig.add_trace(go.Scatter3d(
                x=[goal_x], y=[goal_y], z=[5], 
                mode='markers', marker=dict(size=10, color='blue'), name='Goal'
            ))

            fig.update_layout(
                scene=dict(
                    aspectmode='data', # Keeps proportions real
                    zaxis=dict(range=[0, 20], title="Altitude (m)"),
                    xaxis=dict(title="X (m)"),
                    yaxis=dict(title="Y (m)"),
                ),
                margin=dict(l=0, r=0, b=0, t=0),
                height=600
            )
            
            st.plotly_chart(fig, use_container_width=True)
    else:
        st.info("👈 Add buildings on the right or configure settings on the left, then click 'Run Simulation'.")