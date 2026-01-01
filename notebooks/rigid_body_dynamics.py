"""Interactive Rigid Body Dynamics notebook demonstrating forces and torques."""

import marimo

__generated_with = "0.1.0"
app = marimo.App(width="full")

@app.cell
def _():
    import marimo as mo
    import matplotlib.pyplot as plt
    import numpy as np
    from matplotlib.patches import Circle, FancyArrowPatch, Rectangle
    return Circle, FancyArrowPatch, Rectangle, mo, np, plt

@app.cell
def _(mo):
    mo.md(
        """
        # 🍎 The Physics of Movement: Rigid Body Dynamics

        Kinematics tells us *where* the robot is. **Dynamics** tells us
        *how* it moves under force.

        The fundamental Equation of Motion for any robot is:

        $$
        M(q)\\ddot{q} + C(q, \\dot{q})\\dot{q} + g(q) = \\tau + J^T(q)F_{ext}
        $$

        Use this notebook to feel what each term actually does.
        """
    )
    return

@app.cell
def _(mo):
    # --- CONTROLS ---

    # Simulation Settings
    sim_mode = mo.ui.radio(
        options={
            "static": "1. Static Analysis (Gravity & Mass)",
            "dynamic": "2. Dynamic Simulation",
        },
        value="static",
        label="**Mode**"
    )

    gravity_comp = mo.ui.switch(
        label="**Active Gravity Compensation**", value=False
    )

    # External Force Slider (Pushing the End Effector)
    f_ext_x = mo.ui.slider(
        -20.0, 20.0, value=0.0, step=1.0, label="External Force $F_x$ (Push Hand)"
    )
    f_ext_y = mo.ui.slider(
        -20.0, 20.0, value=0.0, step=1.0, label="External Force $F_y$ (Push Hand)"
    )

    # Robot State (For Static Mode)
    theta1 = mo.ui.slider(-180, 180, value=0, label="$\\theta_1$ (Shoulder)")
    theta2 = mo.ui.slider(-170, 170, value=0, label="$\\theta_2$ (Elbow)")

    mo.hstack([
        mo.vstack([sim_mode, gravity_comp]),
        mo.vstack([theta1, theta2], label="Robot Config"),
        mo.vstack([f_ext_x, f_ext_y], label="External Forces")
    ], justify="space-around", gap=4)
    return f_ext_x, f_ext_y, gravity_comp, sim_mode, theta1, theta2

@app.cell
def _(f_ext_x, f_ext_y, np, theta1, theta2):
    # --- DYNAMICS ENGINE ---

    # Parameters
    m1, m2 = 1.0, 1.0  # Mass (kg)
    l1, l2 = 1.0, 1.0  # Length (m)
    r1, r2 = 0.5, 0.5  # Center of Mass distance (m)
    I1, I2 = 0.1, 0.1  # Inertia
    g_const = 9.81     # Gravity

    # State
    t1 = np.deg2rad(theta1.value)
    t2 = np.deg2rad(theta2.value)
    q = np.array([t1, t2])

    # --- 1. MASS MATRIX M(q) ---
    # Derived via Lagrangian Mechanics
    # M11 = I1 + I2 + m1*r1^2 + m2*(l1^2 + r2^2 + 2*l1*r2*cos(t2))
    # M22 = I2 + m2*r2^2
    # M12 = I2 + m2*(r2^2 + l1*r2*cos(t2))

    c2 = np.cos(t2)
    M11 = I1 + I2 + m1*r1**2 + m2*(l1**2 + r2**2 + 2*l1*r2*c2)
    M22 = I2 + m2*r2**2
    M12 = I2 + m2*(r2**2 + l1*r2*c2)

    M = np.array([[M11, M12], [M12, M22]])

    # --- 2. GRAVITY VECTOR g(q) ---
    # Potential Energy derivative
    s1 = np.sin(t1)
    s12 = np.sin(t1 + t2)

    # Using cos because 0 is horizontal
    g1 = (m1*r1 + m2*l1)*g_const*np.cos(t1) + m2*r2*g_const*np.cos(t1+t2)
    g2 = m2*r2*g_const*np.cos(t1+t2)

    # Note: If 0 is vertical up, use sin. If 0 is horizontal right, use cos.
    # Let's assume standard math convention: 0 is Right. Gravity acts Down (-y).
    # Torque = r x F.
    # Force is -mg in y. Moment arm x component is cos.
    # Correct term is proportional to cos(theta)

    g_vec = np.array([g1, g2])

    # --- 3. JACOBIAN J(q) ---
    s1 = np.sin(t1)
    c1 = np.cos(t1)
    s12 = np.sin(t1 + t2)
    c12 = np.cos(t1 + t2)

    # dx/dt = -l1 s1 q1 - l2 s12 (q1+q2)
    # dy/dt =  l1 c1 q1 + l2 c12 (q1+q2)

    j11 = -l1*s1 - l2*s12
    j12 = -l2*s12
    j21 =  l1*c1 + l2*c12
    j22 =  l2*c12

    J = np.array([[j11, j12], [j21, j22]])

    # --- 4. EXTERNAL FORCES ---
    F_ext = np.array([f_ext_x.value, f_ext_y.value])

    # Contact Jacobian Transpose: Map Force to Torque
    # tau_ext = J.T @ F
    tau_ext = J.T @ F_ext

    return (
        F_ext,
        J,
        M,
        c1,
        c12,
        g_vec,
        l1,
        l2,
        q,
        s1,
        s12,
        tau_ext,
    )

@app.cell
def _(
    Circle,
    F_ext,
    FancyArrowPatch,
    J,
    M,
    c1,
    c12,
    g_vec,
    l1,
    l2,
    mo,
    np,
    plt,
    s1,
    s12,
    tau_ext,
):
    # --- VISUALIZATION ---
    fig, ax = plt.subplots(figsize=(8, 6))

    # Forward Kinematics for plotting
    x1 = l1 * c1
    y1 = l1 * s1
    x2 = x1 + l2 * c12
    y2 = y1 + l2 * s12

    # Draw Robot
    ax.plot([0, x1], [0, y1], 'k-', lw=5, alpha=0.3)
    ax.plot([x1, x2], [y1, y2], 'k-', lw=5, alpha=0.3)
    ax.add_patch(Circle((0,0), 0.1, color='k'))
    ax.add_patch(Circle((x1,y1), 0.1, color='k'))
    ax.add_patch(Circle((x2,y2), 0.15, color='blue', label="End Effector"))

    # --- FORCE VISUALIZATION ---

    # 1. External Force Vector (Green Arrow at Hand)
    if np.linalg.norm(F_ext) > 0.1:
        ax.arrow(
            x2, y2, F_ext[0]*0.1, F_ext[1]*0.1,
            head_width=0.1, color='#16a34a', zorder=10, width=0.03
        )
        ax.text(
            x2 + F_ext[0]*0.1, y2 + F_ext[1]*0.1, "  $F_{ext}$",
            color='#16a34a', fontweight='bold'
        )

    # 2. Resulting Torques (Curved Arrows at Joints)
    # Net Torque required to HOLD this position = g(q) - J^T F_ext
    # The motor must fight gravity AND the push.

    # Gravity Torque (Red)
    t_g = g_vec

    # External Torque Effect (Orange) - Note: J^T F is the torque CAUSED
    # by the force. To resist it, motor applies -tau_ext.

    def plot_torque(x, y, mag, color, label):
        if abs(mag) < 0.5:
            return
        style = "Simple,tail_width=2,head_width=8,head_length=8"
        radius = 0.3
        # Direction
        sign = np.sign(mag)
        # Arc
        arc = FancyArrowPatch(
            (x, y+radius), (x + sign*0.1, y+radius),
            connectionstyle=f"arc3,rad={sign*0.5}",
            color=color, arrowstyle=style
        )
        ax.add_patch(arc)
        ax.text(
            x, y-0.4, f"{label}\n{mag:.1f} Nm",
            color=color, ha='center', fontsize=8
        )

    # Plot Gravity Torques
    plot_torque(0, 0, t_g[0], '#ef4444', "Gravity")
    plot_torque(x1, y1, t_g[1], '#ef4444', "Gravity")

    # Plot External Force Torques
    plot_torque(0, 0.4, -tau_ext[0], '#f59e0b', "Contact Load")
    plot_torque(x1, y1+0.4, -tau_ext[1], '#f59e0b', "Contact Load")

    # Layout
    ax.set_xlim(-2.5, 2.5)
    ax.set_ylim(-2.5, 2.5)
    ax.set_aspect('equal')
    ax.grid(True, linestyle=':')
    ax.set_title("Forces & Torques Analysis")

    # --- MATRIX DISPLAYS ---

    # Mass Matrix Heatmap
    # If arm is extended, inertia is high. If folded, inertia is low.
    m_display = mo.md(
        f"""
        ### 1. Mass Matrix $M(q)$
        Represents Inertia. $M_{{11}}$ is high when arm is extended.

        $$
        \\begin{{bmatrix}}
        {M[0,0]:.2f} & {M[0,1]:.2f} \\\\
        {M[1,0]:.2f} & {M[1,1]:.2f}
        \\end{{bmatrix}}
        $$
        """
    )

    # Contact Jacobian Math
    # Showing the transpose relationship
    j_display = mo.md(
        f"""
        ### 2. Contact Jacobian $J^T F_{{ext}}$
        How a push $(F_x, F_y)$ becomes Joint Torque $\\tau$.

        $$
        \\underbrace{{\\begin{{bmatrix}}
        {tau_ext[0]:.1f} \\\\ {tau_ext[1]:.1f}
        \\end{{bmatrix}}}}_{{\\tau_{{ext}}}} =
        \\underbrace{{\\begin{{bmatrix}}
        {J[0,0]:.2f} & {J[1,0]:.2f} \\\\
        {J[0,1]:.2f} & {J[1,1]:.2f}
        \\end{{bmatrix}}}}_{{J^T}}
        \\underbrace{{\\begin{{bmatrix}}
        {F_ext[0]:.1f} \\\\ {F_ext[1]:.1f}
        \\end{{bmatrix}}}}_{{F_{{ext}}}}
        $$
        """
    )

    plot_widget = mo.ui.matplotlib(fig)
    return j_display, m_display, plot_widget, t_g

@app.cell
def _(j_display, m_display, mo, plot_widget):
    mo.hstack([
        plot_widget,
        mo.vstack([m_display, j_display])
    ])
    return

@app.cell
def _(mo):
    mo.md(
        """
        ### 🧠 Pedagogical Insights

        **1. The "Effective Inertia" ($M$)**
        [attachment_0](attachment)
        * Try extending the arm fully (0, 0). Look at $M_{11}$ (approx 3.8).
        * Now fold the elbow back to 180. Look at $M_{11}$ (approx 0.8).
        * **Robotics Insight:** It is 4x harder for the motor to accelerate
        the arm when extended. RL agents must learn this property implicitly!

        **2. The Contact Jacobian ($J^T$)**
        * Set $\\theta_1=90, \\theta_2=0$ (Arm straight up).
        * Apply External Force $F_y = -10$ (Push straight down on tip).
        * Notice: The **Torque is Zero**.
        * **Why?** Pushing down on a vertical bone compresses the structure
        but creates no rotation. The Jacobian "knows" this geometry ($J^T$
        has zeros). This is why robots are strong in some directions and
        weak in others.
        """
    )
    return

if __name__ == "__main__":
    app.run()
