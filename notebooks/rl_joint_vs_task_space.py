"""RL Tutorial: Joint-Space vs. Task-Space Control - Interactive demonstration."""

import marimo

__generated_with = "0.1.0"
app = marimo.App(width="full")


@app.cell
def __():
    import marimo as mo
    import numpy as np
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, FancyArrowPatch

    return Circle, FancyArrowPatch, mo, np, plt


@app.cell
def __(mo):
    mo.md(
        """
        # 🧠 RL Tutorial: Joint-Space vs. Task-Space Control
        
        In Reinforcement Learning, you must define the **Action Space**: What do the output neurons of your neural network actually control?
        
        This notebook simulates a single time-step $(t)$ in an RL episode. You act as the **Policy Network**. 
        You output a vector $a = [a_1, a_2]$ (via sliders). See how this vector translates to physical movement differently depending on your choice of space.
        """
    )
    return ()


@app.cell
def __(mo):
    # --- CONTROLS ---

    # 1. Action Space Selector
    control_mode = mo.ui.radio(
        options={
            "joint": "Joint Space (Agent controls Motors)",
            "task": "Task Space (Agent controls Coordinates)",
        },
        value="joint",
        label="**1. Choose Action Space**",
    )

    # 2. The "Neural Network" Output (User Sliders)
    # In RL, these are typically normalized outputs between -1 and 1
    action_1 = mo.ui.slider(-1.0, 1.0, step=0.1, value=0.5, label="Action Neuron 1")
    action_2 = mo.ui.slider(-1.0, 1.0, step=0.1, value=0.0, label="Action Neuron 2")

    # 3. Robot State (Configuration)
    # We let the user set the current state to see how actions behave in different poses
    theta1_state = mo.ui.slider(-180, 180, value=10, label="Current State $\\theta_1$")
    theta2_state = mo.ui.slider(-170, 170, value=90, label="Current State $\\theta_2$")

    mo.hstack(
        [
            mo.vstack([control_mode, mo.md("---"), action_1, action_2]),
            mo.vstack([theta1_state, theta2_state]),
        ],
        justify="space-around",
        gap=4,
    )
    return action_1, action_2, control_mode, theta1_state, theta2_state


@app.cell
def __(action_1, action_2, control_mode, np, theta1_state, theta2_state):
    # --- SIMULATION ENGINE ---

    # Robot Parameters
    L1, L2 = 3.0, 2.5

    # Current State (Converted to radians)
    t1 = np.deg2rad(theta1_state.value)
    t2 = np.deg2rad(theta2_state.value)
    t12 = t1 + t2

    # Forward Kinematics (Where are we now?)
    j1_x = L1 * np.cos(t1)
    j1_y = L1 * np.sin(t1)
    ee_x = j1_x + L2 * np.cos(t12)
    ee_y = j1_y + L2 * np.sin(t12)

    # Jacobian Calculation (Needed for dynamics)
    j11 = -L1 * np.sin(t1) - L2 * np.sin(t12)
    j12 = -L2 * np.sin(t12)
    j21 = L1 * np.cos(t1) + L2 * np.cos(t12)
    j22 = L2 * np.cos(t12)
    J = np.array([[j11, j12], [j21, j22]])

    # --- PROCESS ACTION ---

    raw_action = np.array([action_1.value, action_2.value])

    # Variables to store the "Resulting Velocity"
    # We want to know: What happens to the Joints (q_dot)? What happens to the Hand (p_dot)?

    q_dot = np.zeros(2)  # Joint Velocity
    p_dot = np.zeros(2)  # End Effector Velocity

    if control_mode.value == "joint":
        # Case A: Joint Space Control
        # The agent directly sets angular velocity.
        # Action [0.5, 0.2] means "Rotate Shoulder at 0.5 rad/s, Elbow at 0.2 rad/s"
        q_dot = raw_action * 2.0  # Scaling factor for visibility

        # We calculate resulting hand movement using Forward Kinematics (Jacobian)
        p_dot = J @ q_dot

    else:
        # Case B: Task Space Control
        # The agent sets Cartesian velocity.
        # Action [0.5, 0.2] means "Move Hand Right at 0.5 m/s, Up at 0.2 m/s"
        p_dot = raw_action * 2.0  # Scaling factor

        # We calculate required joint movement using Inverse Kinematics (Pseudo-Inverse)
        try:
            J_inv = np.linalg.pinv(J)
            q_dot = J_inv @ p_dot
        except:
            q_dot = np.zeros(2)

    return (
        J,
        L1,
        L2,
        ee_x,
        ee_y,
        j1_x,
        j1_y,
        p_dot,
        q_dot,
        raw_action,
        t1,
        t2,
    )


@app.cell
def __(
    Circle,
    FancyArrowPatch,
    L1,
    L2,
    control_mode,
    ee_x,
    ee_y,
    j1_x,
    j1_y,
    mo,
    np,
    p_dot,
    plt,
    q_dot,
    t1,
):
    # --- VISUALIZATION ---

    fig, ax = plt.subplots(figsize=(8, 6))

    # Draw Robot
    ax.plot([0, j1_x], [0, j1_y], color="#94a3b8", lw=4, solid_capstyle="round")
    ax.plot([j1_x, ee_x], [j1_y, ee_y], color="#94a3b8", lw=4, solid_capstyle="round")
    ax.add_patch(Circle((0, 0), 0.2, color="#475569"))
    ax.add_patch(Circle((j1_x, j1_y), 0.2, color="white", ec="#475569", lw=2))
    ax.add_patch(Circle((ee_x, ee_y), 0.25, color="#0f172a"))

    # Draw "Ghost" Prediction (Where it will be in delta_t)
    dt = 0.5
    # For visualization, we approximate next pos
    # Note: This is a linear approximation. Real dynamics are curved.
    pred_ee_x = ee_x + p_dot[0] * dt
    pred_ee_y = ee_y + p_dot[1] * dt

    ax.plot([ee_x, pred_ee_x], [ee_y, pred_ee_y], "k:", alpha=0.3)
    ax.add_patch(
        Circle(
            (pred_ee_x, pred_ee_y), 0.15, color="#22c55e", alpha=0.6, label="Next Step State"
        )
    )

    # --- ACTION VECTORS ---

    if control_mode.value == "joint":
        # Visualize Joint Space Actions (Rotational Arrows)
        color = "#d946ef"  # Magenta for Actions

        # Joint 1 Action
        if abs(q_dot[0]) > 0.1:
            ax.text(
                0,
                -0.6,
                f"Action 1:\n{q_dot[0]:.2f} rad/s",
                color=color,
                ha="center",
                fontweight="bold",
            )
            # Draw curved arrow approximation
            offset = 0.2 if q_dot[0] > 0 else -0.2
            ax.arrow(0.5, 0, 0, offset, color=color, head_width=0.1)

        # Joint 2 Action
        if abs(q_dot[1]) > 0.1:
            ax.text(
                j1_x,
                j1_y - 0.6,
                f"Action 2:\n{q_dot[1]:.2f} rad/s",
                color=color,
                ha="center",
                fontweight="bold",
            )
            ax.arrow(
                j1_x + 0.5, j1_y, 0, 0.2 if q_dot[1] > 0 else -0.2, color=color, head_width=0.1
            )

        # Resulting EE movement (Consequence)
        ax.arrow(
            ee_x,
            ee_y,
            p_dot[0],
            p_dot[1],
            color="#64748b",
            head_width=0.15,
            width=0.05,
            alpha=0.5,
        )
        ax.text(ee_x, ee_y + 0.5, "Resulting Motion", color="#64748b", fontsize=8)

    else:
        # Visualize Task Space Actions (Linear Arrows)
        color = "#d946ef"  # Magenta for Actions

        # The Action is directly the EE vector
        ax.arrow(
            ee_x, ee_y, p_dot[0], p_dot[1], color=color, head_width=0.2, width=0.08, zorder=10
        )
        ax.text(
            ee_x + p_dot[0],
            ee_y + p_dot[1] + 0.2,
            "Action Vector",
            color=color,
            fontweight="bold",
        )

        # Resulting Joint movements (The Solver's work)
        # We visualize these as ghost arrows to show what the "Lower Level Controller" is doing
        if abs(q_dot[0]) > 0.05:
            ax.text(
                0, -0.6, f"Solver:\n{q_dot[0]:.2f} rad/s", color="#64748b", ha="center", fontsize=9
            )
        if abs(q_dot[1]) > 0.05:
            ax.text(
                j1_x,
                j1_y - 0.6,
                f"Solver:\n{q_dot[1]:.2f} rad/s",
                color="#64748b",
                ha="center",
                fontsize=9,
            )

    # Chart Settings
    ax.set_xlim(-6, 6)
    ax.set_ylim(-4, 6)
    ax.set_aspect("equal")
    ax.grid(True, linestyle=":", alpha=0.5)
    ax.set_title(f"Episode Step: $t \\to t+1$", fontsize=14)
    ax.legend(loc="upper left")

    viz_chart = mo.ui.matplotlib(fig)
    return ax, dt, fig, pred_ee_x, pred_ee_y, viz_chart


@app.cell
def __(control_mode, mo, p_dot, q_dot, raw_action):
    # --- EXPLANATION & PEDAGOGY ---

    if control_mode.value == "joint":
        explanation = mo.md(
            rf"""
            ### 🤖 Mode: Joint Space Control
            
            **The Agent says:** "Move Shoulder speed {raw_action[0]}, Elbow speed {raw_action[1]}"
            
            **The Math:**
            $$ a_t = \dot{{q}} $$
            The end-effector velocity $\dot{{p}}$ is a *consequence* of this action via the Forward Kinematics.
            
            **RL Implications:**
            * **Pros:** The agent has full control. No "domain errors" or singularities. If the arm can physically move, the agent can make it move.
            * **Cons:** **Non-Linearity.** Look at the plot. To move the hand in a straight line, the agent must learn a complex, varying ratio of shoulder/elbow speeds. This makes the learning curve steeper.
            """
        ).callout(kind="info")
    else:
        explanation = mo.md(
            rf"""
            ### 🎯 Mode: Task Space Control
            
            **The Agent says:** "Move Hand Right speed {raw_action[0]}, Up speed {raw_action[1]}"
            
            **The Math:**
            $$ a_t = \dot{{p}}_{{desired}} $$
            The motor velocities are calculated by a solver (Inverse Kinematics): $\dot{{q}} = J^{{-1}} a_t$
            
            **RL Implications:**
            * **Pros:** **Linearity.** If the reward is "Move to the target", the mapping is 1:1. The agent learns very quickly.
            * **Cons:** **Singularities.** Try this: Fully extend the arm (State: 0, 0) and try to move "Out" (Action: 1.0, 0.0). The Green Dot won't move! The math fails ($J$ is singular). The agent outputs an action, but the robot does nothing. This can "kill" the learning process (Vanishing Gradient).
            """
        ).callout(kind="success")

    debug_info = mo.md(
        rf"""
        **System State:**
        * **Action Vector:** {raw_action}
        * **Joint Velocities ($\dot{{q}}$):** [{q_dot[0]:.2f}, {q_dot[1]:.2f}]
        * **Hand Velocity ($\dot{{p}}$):** [{p_dot[0]:.2f}, {p_dot[1]:.2f}]
        """
    )

    mo.vstack([explanation, debug_info], gap=2)
    return debug_info, explanation


@app.cell
def __(mo, viz_chart):
    mo.hstack([viz_chart], justify="center")
    return ()


if __name__ == "__main__":
    app.run()
