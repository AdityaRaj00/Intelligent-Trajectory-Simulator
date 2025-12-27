import streamlit as st
from streamlit.components.v1 import html

ga_code = """
<script async src="https://www.googletagmanager.com/gtag/js?id=G-9D24G48BZT"></script>
<script>
  window.dataLayer = window.dataLayer || [];
  function gtag(){dataLayer.push(arguments);}

  gtag('js', new Date());

  gtag('config', 'G-9D24G48BZT', {
    'send_page_view': false
  });
  setTimeout(function() {
    gtag('event', 'streamlit_page_loaded', {
      'event_category': 'engagement',
      'event_label': 'trajectory_simulator'
    });
  }, 1000);
</script>
"""

html(ga_code, height=0)


import plotly.graph_objects as go
import numpy as np
import random
import time
from environment import Airspace
from planner import RoutePlanner
import math

st.set_page_config(page_title="Intelligent Trajectory Simulator", layout="wide")
st.title("🚁 3D Trajectory Optimizer for UAS")

def safe_rerun():
    if hasattr(st, "rerun"):
        st.rerun()
    elif hasattr(st, "experimental_rerun"):
        st.experimental_rerun()
    else:
        st.info("Please refresh the page.")

st.session_state.setdefault("custom_obstacles", [])
st.session_state.setdefault("last_path", None)
st.session_state.setdefault("last_metrics", None)

with st.sidebar:
    st.header("Simulation Parameters")
    grid_size = st.slider("Map Size (meters)", 20, 80, 30)

    st.subheader("Weather")
    wind_alt = st.slider("Wind Start Altitude", 1, 20, 10)
    wind_speed = st.slider("Wind Speed (m/s)", 0.0, 20.0, 4.0)
    wind_dir = st.slider("Wind Direction (°)", 0, 360, 180)

    st.subheader("Physics")
    climb_cost = st.slider("Climb Cost", 0.1, 5.0, 2.0)
    safety_penalty = st.slider("Low Altitude Penalty", 0.0, 30.0, 8.0)

    st.subheader("Waypoints")
    c1, c2 = st.columns(2)
    with c1:
        start_x = st.number_input("Start X", 0, grid_size-1, 2)
        start_y = st.number_input("Start Y", 0, grid_size-1, 2)
    with c2:
        goal_x = st.number_input("Goal X", 0, grid_size-1, grid_size-2)
        goal_y = st.number_input("Goal Y", 0, grid_size-1, grid_size-2)

    st.subheader("Wind Vector Visualization")
    show_wind = st.checkbox("Show wind vectors (cones)", True)
    wind_step = st.slider("Wind sampling step (higher = fewer cones)", 4, max(4, grid_size//2), max(6, grid_size//6))
    wind_sizeref = st.slider("Wind cone size reference (sizeref)", 0.5, 6.0, 2.0)

def serialize_obs(obs_list):
    return tuple((o["x"], o["y"], o["dx"], o["dy"], o["dz"]) for o in obs_list)

def deserialize_obs(serialized):
    return [{"x": a, "y": b, "dx": c, "dy": d, "dz": e} for (a, b, c, d, e) in serialized]

def load_sample(name, grid):
    st.session_state.custom_obstacles = []
    if name == "City (dense)":
        for i in range(5, min(grid, 25), 4):
            for j in range(5, min(grid, 25), 4):
                st.session_state.custom_obstacles.append(
                    {"x": i, "y": j, "dx": 3, "dy": 3, "dz": random.randint(6, 14)}
                )
    elif name == "Sparse Field":
        for _ in range(8):
            x = random.randint(0, max(0, grid - 5))
            y = random.randint(0, max(0, grid - 5))
            st.session_state.custom_obstacles.append(
                {"x": x, "y": y, "dx": random.randint(2,4), "dy": random.randint(2,4), "dz": random.randint(5,12)}
            )

col_viz, col_tools = st.columns([3, 1])

with col_tools:
    st.subheader("Obstacles")
    tab1, tab2 = st.tabs(["Manual Add", "Random Gen"])

    with tab1:
        with st.form("manual"):
            ox = st.number_input("X", 0, grid_size, 10)
            oy = st.number_input("Y", 0, grid_size, 10)
            dx = st.number_input("Width", 1, 10, 4)
            dy = st.number_input("Length", 1, 10, 4)
            dz = st.number_input("Height", 1, 20, 10)
            if st.form_submit_button("Add"):
                st.session_state.custom_obstacles.append(
                    {"x": int(ox), "y": int(oy), "dx": int(dx), "dy": int(dy), "dz": int(dz)}
                )

    with tab2:
        with st.form("random"):
            count = st.number_input("Count", 1, 100, 8)
            max_h = st.number_input("Max Height", 5, 40, 12)
            if st.form_submit_button("Generate"):
                for _ in range(int(count)):
                    x = random.randint(0, max(0, grid_size - 5))
                    y = random.randint(0, max(0, grid_size - 5))
                    st.session_state.custom_obstacles.append(
                        {"x": x, "y": y, "dx": random.randint(2,5), "dy": random.randint(2,5), "dz": random.randint(5, max_h)}
                    )

    st.divider()
    sample = st.selectbox("Sample scenario", ["(none)", "City (dense)", "Sparse Field"])
    if st.button("Load") and sample != "(none)":
        load_sample(sample, grid_size)
        safe_rerun()

    if st.session_state.custom_obstacles:
        st.write(f"Total buildings: {len(st.session_state.custom_obstacles)}")
        if st.button("Clear"):
            st.session_state.custom_obstacles = []
            safe_rerun()

@st.cache_data
def run_sim(grid_size, wind_alt, wind_speed, wind_dir, climb_cost, safety_penalty, serialized_obs, start_node, goal_node):
    obstacles = deserialize_obs(serialized_obs)
    env = Airspace((grid_size, grid_size, 20))
    env.set_wind(z_level=wind_alt, intensity=wind_speed, direction_degrees=wind_dir)

    for o in obstacles:
        x, y, dx, dy, dz = int(o["x"]), int(o["y"]), int(o["dx"]), int(o["dy"]), int(o["dz"])
        if x + dx <= grid_size and y + dy <= grid_size:
            env.grid[x:x+dx, y:y+dy, 0:dz] = 1

    planner = RoutePlanner(env, {
        "climb_cost": climb_cost,
        "wind_penalty": wind_speed,
        "safety_penalty": safety_penalty
    })

    return planner.find_path(start_node, goal_node)  # (path, metrics)

def segment_breakdown(a, b, climb_cost, wind_penalty, safety_penalty):
    ax, ay, az = a
    bx, by, bz = b
    horiz = math.hypot(bx-ax, by-ay)
    climb = max(0.0, bz - az)
    safety = safety_penalty * (1.0 / (min(az, bz) + 1.0))
    wind = wind_penalty * (horiz / (1.0 + az))
    horiz_energy = horiz
    climb_energy = climb_cost * climb
    total = horiz_energy + climb_energy + safety + wind
    return {"horiz": horiz_energy, "climb": climb_energy, "safety": safety, "wind": wind, "total": total}

with col_viz:
    show_segment_info = st.checkbox("Show per-segment energy hover info", True)

    if st.button("🚀 Run Simulation"):
        with st.spinner("Computing optimal path..."):
            serialized = serialize_obs(st.session_state.custom_obstacles)
            start_node = (int(start_x), int(start_y), 0)
            goal_node = (int(goal_x), int(goal_y), 5)

            path, metrics = run_sim(
                grid_size, wind_alt, wind_speed, wind_dir,
                climb_cost, safety_penalty, serialized,
                start_node, goal_node
            )

            st.session_state.last_path = path
            st.session_state.last_metrics = metrics

            # Build env for plotting
            env = Airspace((grid_size, grid_size, 20))
            env.set_wind(z_level=wind_alt, intensity=wind_speed, direction_degrees=wind_dir)
            for o in deserialize_obs(serialized):
                env.grid[o["x"]:o["x"]+o["dx"], o["y"]:o["y"]+o["dy"], :o["dz"]] = 1

            fig = go.Figure()
            ox, oy, oz = np.where(env.grid == 1)
            fig.add_trace(go.Scatter3d(
                x=ox, y=oy, z=oz, mode="markers",
                marker=dict(size=3, opacity=0.4), name="Buildings"
            ))

            # Path trace with optional hover breakdown per node (shows incoming segment)
            if path:
                px, py, pz = zip(*path)
                hover_text = []
                # first node has no incoming segment
                hover_text.append("start")
                for i in range(1, len(path)):
                    bd = segment_breakdown(path[i-1], path[i], climb_cost, wind_speed, safety_penalty)
                    if show_segment_info:
                        hover_text.append(
                            f"seg {i}: total={bd['total']:.2f}<br>"
                            f"horiz={bd['horiz']:.2f}, climb={bd['climb']:.2f}<br>"
                            f"safety={bd['safety']:.2f}, wind={bd['wind']:.2f}"
                        )
                    else:
                        hover_text.append(f"seg {i}: total={bd['total']:.2f}")

                fig.add_trace(go.Scatter3d(
                    x=px, y=py, z=pz, mode="lines+markers",
                    line=dict(color="red", width=5), marker=dict(size=4, color="yellow"),
                    name="Path", hoverinfo="text", hovertext=hover_text
                ))
            else:
                st.error("No path found.")

            # Wind vectors (cones) with user-controlled density & size
            if show_wind and wind_speed > 0.01:
                theta = np.deg2rad(wind_dir)
                ux = math.cos(theta)
                uy = math.sin(theta)
                uz = 0.0
                xs, ys, zs, us, vs, ws = [], [], [], [], [], []
                step = max(1, int(wind_step))
                for xi in range(0, grid_size, step):
                    for yi in range(0, grid_size, step):
                        xs.append(xi + 0.5)
                        ys.append(yi + 0.5)
                        zs.append(min(max(1, wind_alt + 1), 19))
                        scale = max(0.1, float(wind_speed))
                        us.append(ux * scale)
                        vs.append(uy * scale)
                        ws.append(0.0)

                if any(us):
                    fig.add_trace(go.Cone(
                        x=xs, y=ys, z=zs, u=us, v=vs, w=ws,
                        sizemode="absolute", sizeref=wind_sizeref,
                        anchor="tail", showscale=False, name="Wind Vectors"
                    ))

            fig.update_layout(
                scene=dict(aspectmode="data",
                           xaxis=dict(title="X"),
                           yaxis=dict(title="Y"),
                           zaxis=dict(title="Z")),
                height=700, margin=dict(l=0, r=0, t=0, b=0)
            )

            st.plotly_chart(fig, use_container_width=True)

    # Export & metrics
    st.subheader("Export")
    if st.session_state.last_path:
        csv = "x,y,z\n" + "\n".join(f"{x},{y},{z}" for x,y,z in st.session_state.last_path)
        st.download_button("Download CSV", csv, "trajectory.csv")

    st.subheader("Run Metrics")
    if st.session_state.last_metrics:
        m = st.session_state.last_metrics
        c1, c2, c3 = st.columns(3)
        c1.metric("Time (s)", f"{m['time_s']:.3f}")
        c2.metric("Cost", f"{m['cost']:.2f}")
        c3.metric("Nodes", m["nodes_explored"])

        bd = m.get("breakdown", {})
        st.markdown("**Energy Cost Breakdown**")
        bcols = st.columns(4)
        bcols[0].metric("Horizontal", f"{bd.get('horiz',0):.2f}")
        bcols[1].metric("Climb", f"{bd.get('climb',0):.2f}")
        bcols[2].metric("Safety", f"{bd.get('safety',0):.2f}")
        bcols[3].metric("Wind", f"{bd.get('wind',0):.2f}")

        bar_fig = go.Figure(go.Bar(x=["horiz","climb","safety","wind"], y=[bd.get(k,0.0) for k in ["horiz","climb","safety","wind"]]))
        bar_fig.update_layout(margin=dict(l=0,r=0,t=10,b=0), height=220)
        st.plotly_chart(bar_fig, use_container_width=True)
    else:
        st.caption("Run a simulation to see runtime, cost and energy breakdown.")
