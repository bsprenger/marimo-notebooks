"""Interactive notebook visualizing the SO-100 planar robot from geometry to control."""

import marimo

__generated_with = "0.1.0"
app = marimo.App(width="full")


@app.cell
def __():
    import marimo as mo
    import numpy as np
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle, FancyArrowPatch
    return Circle, FancyArrowPatch, Rectangle, mo, np, plt


@app.cell
def __(mo):
    mo.md(
        """
        # 📚 From Geometry to Control: The Math of the SO-100
        
        This interactive notebook visualizes the mathematical concepts behind manipulating a 2-DOF planar robot, 
        specifically illustrating the transition from **Forward Kinematics** to **Differential IK**.
        """
    )
    return ()


@app.cell
def __(mo):
    # --- CONTROLS ---
    
    # Scene Controls
    constraint_mode = mo.ui.dropdown(
        options={"free": "Free Space", "floor": "Floor Constraint", "obstacle": "Floor + Obstacle"},
        value="free",
        label="**Environment (Constraint Set $Q$)**"
    )
    
    vis_mode = mo.ui.radio(
        options={"fk": "1. Geometry (FK)", "diff": "2. Differential Kinematics (Jacobian)"},
        value="fk",
        label="**Visualization Mode**"
    )

    # Robot Controls
    theta1 = mo.ui.slider(-180, 180, value=45, step=1, label=r"$\theta_1$ (Shoulder)")
    theta2 = mo.ui.slider(-170, 170, value=-45, step=1, label=r"$\theta_2$ (Elbow)")
    
    # Diff IK Target Velocity Controls
    vel_x = mo.ui.slider(-1.0, 1.0, step=0.1, value=0.5, label=r"Desired $\dot{p}_x$")
    vel_y = mo.ui.slider(-1.0, 1.0, step=0.1, value=0.0, label=r"Desired $\dot{p}_y$")

    mo.hstack([
        mo.vstack([constraint_mode, vis_mode]),
        mo.vstack([theta1, theta2]),
        mo.vstack([vel_x, vel_y])
    ], justify="space-around", gap=4)
    return constraint_mode, theta1, theta2, vel_x, vel_y, vis_mode


@app.cell
def __(constraint_mode, np, theta1, theta2, vel_x, vel_y):
    # --- KINEMATICS ENGINE ---
    
    # Constants for SO-100 Planar Model
    L1 = 3.0
    L2 = 2.5
    
    # Convert inputs to radians
    t1 = np.deg2rad(theta1.value)
    t2 = np.deg2rad(theta2.value)
    t12 = t1 + t2
    
    # 1. Forward Kinematics (Position)
    # Joint 1 position
    j1_x = L1 * np.cos(t1)
    j1_y = L1 * np.sin(t1)
    
    # End Effector position
    ee_x = j1_x + L2 * np.cos(t12)
    ee_y = j1_y + L2 * np.sin(t12)
    
    # 2. Jacobian Calculation (For Diff-IK)
    # J = [[dx/dt1, dx/dt2],
    #      [dy/dt1, dy/dt2]]
    # Derived from derivatives of FK equations
    j11 = -L1 * np.sin(t1) - L2 * np.sin(t12)
    j12 = -L2 * np.sin(t12)
    j21 =  L1 * np.cos(t1) + L2 * np.cos(t12)
    j22 =  L2 * np.cos(t12)
    
    J = np.array([[j11, j12], [j21, j22]])
    
    # 3. Differential IK Solver
    # q_dot = J_pseudo_inv * p_dot_star
    target_vel = np.array([vel_x.value, vel_y.value])
    try:
        J_inv = np.linalg.pinv(J)
        q_dot = J_inv @ target_vel # [omega1, omega2]
    except:
        q_dot = np.array([0.0, 0.0])

    # 4. Collision Detection (The Constraints)
    collision = False
    
    # Floor Constraint (y < 0)
    if constraint_mode.value in ["floor", "obstacle"]:
        if j1_y < 0 or ee_y < 0:
            collision = True
            
    # Obstacle Constraint (Box at x=2, y=0.5)
    obs_x, obs_y, obs_w, obs_h = 2.5, -1.0, 2.0, 3.0 # A shelf
    if constraint_mode.value == "obstacle":
        # Simple point check for elbow and EE
        if (obs_x < j1_x < obs_x + obs_w and obs_y < j1_y < obs_y + obs_h) or \
           (obs_x < ee_x < obs_x + obs_w and obs_y < ee_y < obs_y + obs_h):
            collision = True

    return (
        J,
        L1,
        L2,
        collision,
        ee_x,
        ee_y,
        j1_x,
        j1_y,
        obs_h,
        obs_w,
        obs_x,
        obs_y,
        q_dot,
        t1,
        t12,
        t2,
        target_vel,
    )


@app.cell
def __(
    Circle,
    FancyArrowPatch,
    J,
    L1,
    L2,
    Rectangle,
    collision,
    constraint_mode,
    ee_x,
    ee_y,
    j1_x,
    j1_y,
    mo,
    np,
    obs_h,
    obs_w,
    obs_x,
    obs_y,
    plt,
    q_dot,
    t1,
    t12,
    t2,
    target_vel,
    vis_mode,
):
    # --- VISUALIZATION ---
    fig, ax = plt.subplots(figsize=(8, 6))
    
    # Colors
    robot_col = '#ef4444' if collision else '#2563eb'
    link_col = '#93c5fd'
    
    # 1. Draw Environment
    if constraint_mode.value in ["floor", "obstacle"]:
        # Draw Floor
        ax.axhline(0, color='#374151', linewidth=2)
        ax.fill_between([-10, 10], -10, 0, color='#f3f4f6', alpha=0.5)
        ax.text(-5, -0.5, "Surface Constraint", color='#374151', fontweight='bold')
        
    if constraint_mode.value == "obstacle":
        # Draw Shelf/Obstacle
        rect = Rectangle((obs_x, obs_y), obs_w, obs_h, color='#b91c1c', alpha=0.3)
        ax.add_patch(rect)
        ax.text(obs_x + 0.2, obs_y + obs_h + 0.2, "Obstacle", color='#b91c1c')

    # 2. Draw Robot
    # Links
    ax.plot([0, j1_x], [0, j1_y], color=robot_col, lw=5, solid_capstyle='round') # Link 1
    ax.plot([j1_x, ee_x], [j1_y, ee_y], color=robot_col, lw=5, solid_capstyle='round') # Link 2
    
    # Joints (Spheres as per text)
    ax.add_patch(Circle((0, 0), 0.2, color='#1f2937', zorder=10)) # Base
    ax.add_patch(Circle((j1_x, j1_y), 0.2, color='white', ec='#1f2937', lw=2, zorder=10)) # Elbow
    ax.add_patch(Circle((ee_x, ee_y), 0.2, color=robot_col, zorder=10)) # EE

    # --- MODE SPECIFIC OVERLAYS ---
    
    if vis_mode.value == "fk":
        # Visualizing the Math of FK
        # Draw components for Link 1
        ax.plot([0, j1_x], [0, 0], 'k:', alpha=0.5)
        ax.plot([j1_x, j1_x], [0, j1_y], 'k:', alpha=0.5)
        
        # Draw components for Link 2 (starting from elbow)
        ax.plot([j1_x, ee_x], [j1_y, j1_y], 'k:', alpha=0.5)
        ax.plot([ee_x, ee_x], [j1_y, ee_y], 'k:', alpha=0.5)
        
        # Labels
        mid_l1_x = j1_x / 2
        ax.text(mid_l1_x, -0.4, r"$l_1 \cos(\theta_1)$", fontsize=9, color='#4b5563', ha='center')
        
        mid_l2_x = j1_x + (ee_x - j1_x)/2
        ax.text(mid_l2_x, j1_y - 0.4, r"$l_2 \cos(\theta_1 + \theta_2)$", fontsize=9, color='#4b5563', ha='center')

    elif vis_mode.value == "diff":
        # Visualizing Velocities (Diff IK)
        
        # 1. Draw Target Velocity Vector at EE
        # Scale factor for visibility
        scale = 2.0 
        
        # Target Vector (Green)
        ax.arrow(ee_x, ee_y, target_vel[0]*scale, target_vel[1]*scale, 
                 head_width=0.2, color='#16a34a', zorder=20, label=r"$\dot{p}^*$ (Target)")
        
        # 2. Draw Resulting Joint Velocities (Rotational Arrows)
        # Omega 1 (at base)
        if abs(q_dot[0]) > 0.1:
            style = "Simple,tail_width=0.5,head_width=4,head_length=8"
            rad = 0.5
            start_ang = 0
            end_ang = 45 * np.sign(q_dot[0]) * (abs(q_dot[0])/2) # scale arc length by speed
            
            # Using simple markers for rotation direction
            color = '#9333ea' # Purple for joint velocity
            ax.text(0, 0.5, r"$\dot{\theta}_1$" + f"={q_dot[0]:.2f}", color=color, fontweight='bold')
            # Arc logic is complex in mpl, using simple curved arrow concept:
            # Just plotting a tangent vector for visual simplicity
            tan_x = -np.sin(t1) * q_dot[0]
            tan_y = np.cos(t1) * q_dot[0]
            ax.arrow(j1_x/2, j1_y/2, tan_x, tan_y, color=color, head_width=0.1)

        # Omega 2 (at elbow)
        if abs(q_dot[1]) > 0.1:
            color = '#9333ea'
            ax.text(j1_x, j1_y+0.5, r"$\dot{\theta}_2$" + f"={q_dot[1]:.2f}", color=color, fontweight='bold')
            
    # Chart Setup
    ax.set_xlim(-6, 6)
    ax.set_ylim(-2, 6)
    ax.set_aspect('equal')
    ax.grid(True, linestyle='--', alpha=0.3)
    ax.set_title("SO-100 Planar Model", fontsize=14)
    
    vis_chart = mo.ui.matplotlib(fig)
    return link_col, mid_l1_x, mid_l2_x, rect, robot_col, scale, vis_chart


@app.cell
def __(
    J,
    L1,
    L2,
    collision,
    constraint_mode,
    ee_x,
    ee_y,
    mo,
    np,
    q_dot,
    t1,
    t12,
    t2,
    target_vel,
    vis_mode,
):
    # --- NARRATIVE ENGINE ---
    
    # 1. FK TEXT
    if vis_mode.value == "fk":
        formula = mo.md(
            f"""
            ### 📐 The Forward Kinematics Formula
            
            The text states:
            > "we can analytically write the end-effector's position $p$ as a function of the robot's configuration $q$..."
            
            Look at the **dotted lines** in the graph. They represent the geometric decomposition:
            
            $$
            p_x = \\underbrace{{ {L1}\\cos({np.rad2deg(t1):.0f}^\\circ) }}_{{Link 1}} + \\underbrace{{ {L2}\\cos({np.rad2deg(t12):.0f}^\\circ) }}_{{Link 2}} = \\mathbf{{{ee_x:.2f}}}
            $$
            
            $$
            p_y = \\underbrace{{ {L1}\\sin({np.rad2deg(t1):.0f}^\\circ) }}_{{Link 1}} + \\underbrace{{ {L2}\\sin({np.rad2deg(t12):.0f}^\\circ) }}_{{Link 2}} = \\mathbf{{{ee_y:.2f}}}
            $$
            """
        )
        
        status = mo.md("")
        if collision:
            status = mo.md(
                """
                ### 🚨 Constraint Violation!
                > "the feasible set of solutions Q may change across problems"
                
                The configuration $q$ is currently **invalid** because the robot has hit the environment. 
                In a mathematical solver, this $q$ would be excluded from the set $Q$.
                """
            ).callout(kind="danger")
        else:
             status = mo.md(
                f"""
                ### ✅ Valid Configuration
                Current state $q = [{np.rad2deg(t1):.0f}^\\circ, {np.rad2deg(t2):.0f}^\\circ]$ is inside the feasible set $Q$.
                """
            ).callout(kind="success")
            
        details = mo.vstack([formula, status])

    # 2. DIFF IK TEXT
    else:
        formula = mo.md(
            f"""
            ### 🚀 Differential Inverse Kinematics
            
            Instead of solving for position, we solve for **speed**. 
            We want the End Effector to move with velocity $\\dot{{p}}^* = [{target_vel[0]}, {target_vel[1]}]$.
            
            The **Jacobian Matrix $J(q)$** relates joint speed to hand speed:
            
            $$
            \\begin{{bmatrix}} {J[0,0]:.2f} & {J[0,1]:.2f} \\\\ {J[1,0]:.2f} & {J[1,1]:.2f} \\end{{bmatrix}} 
            \\begin{{bmatrix}} \\dot{{\\theta}}_1 \\\\ \\dot{{\\theta}}_2 \\end{{bmatrix}} = 
            \\begin{{bmatrix}} \\dot{{p}}_x \\\\ \\dot{{p}}_y \\end{{bmatrix}}
            $$
            
            To achieve your target velocity (Green Arrow), we invert $J$ to find the required joint speeds (Purple Arrows):
            
            $$
            \\dot{{\\theta}}_1 = \\mathbf{{{q_dot[0]:.2f}}} \\text{{ rad/s}}, \\quad \\dot{{\\theta}}_2 = \\mathbf{{{q_dot[1]:.2f}}} \\text{{ rad/s}}
            $$
            """
        )
        
        explanation = mo.md(
            """
            > "solving for $\\dot{q}$ is much less dependent on the environment... and admits the closed-form solution via Pseudo-inverse"
            
            **Try this:**
            1. Set the Arm to be straight up (90, 0).
            2. Set Desired $\\dot{p}_x$ to 1.0 (Move Right).
            3. Notice how **Link 1** has to rotate clockwise (-), but **Link 2** rotates counter-clockwise (+) to keep the y-height steady? The Jacobian calculates that ratio instantly.
            """
        ).callout(kind="info")
        
        details = mo.vstack([formula, explanation])

    mo.hstack([vis_chart, details], gap=4)
    return details, explanation, formula, status


if __name__ == "__main__":
    app.run()
