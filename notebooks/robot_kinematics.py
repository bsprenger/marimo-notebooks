"""Interactive Robot Kinematics notebook demonstrating FK and IK."""

import marimo

__generated_with = "0.1.0"
app = marimo.App(width="full")


@app.cell
def __():
    import marimo as mo
    import matplotlib.pyplot as plt
    import numpy as np
    from matplotlib.patches import Circle, Wedge

    return Circle, Wedge, mo, np, plt


@app.cell
def __(mo):
    mo.md(
        """
        # 🤖 Interactive Robot Kinematics
        **Forward Kinematics (FK)**: You set the angles,
        we calculate the position.

        **Inverse Kinematics (IK)**: You set the target position,
        we calculate the angles.
        """
    )
    return ()


@app.cell
def __(mo):
    # --- CONTROLS ---
    mode_switch = mo.ui.switch(label="**Mode: Inverse Kinematics (IK)**", value=False)

    # Link Lengths
    l1_slider = mo.ui.slider(1.0, 5.0, step=0.5, value=3.0, label="L1 Length")
    l2_slider = mo.ui.slider(1.0, 5.0, step=0.5, value=2.0, label="L2 Length")

    # FK Controls
    theta1_slider = mo.ui.slider(-180, 180, value=45, label=r"$\theta_1$ (degrees)")
    theta2_slider = mo.ui.slider(-170, 170, value=45, label=r"$\theta_2$ (degrees)")

    # IK Controls
    target_x = mo.ui.number(-10, 10, step=0.1, value=3.0, label="Target X")
    target_y = mo.ui.number(-10, 10, step=0.1, value=3.0, label="Target Y")

    return (
        l1_slider,
        l2_slider,
        mode_switch,
        target_x,
        target_y,
        theta1_slider,
        theta2_slider,
    )


@app.cell
def __(
    l1_slider,
    l2_slider,
    mode_switch,
    np,
    target_x,
    target_y,
    theta1_slider,
    theta2_slider,
):
    # --- CALCULATION ENGINE ---

    # Extract values
    L1 = l1_slider.value
    L2 = l2_slider.value
    is_ik = mode_switch.value

    # State Variables
    t1_val, t2_val = 0.0, 0.0
    x_val, y_val = 0.0, 0.0
    valid_ik = True

    # Second solution storage (Ghost arm)
    t1_alt, t2_alt = None, None

    if not is_ik:
        # --- FORWARD KINEMATICS ---
        t1_val = np.deg2rad(theta1_slider.value)
        t2_val = np.deg2rad(theta2_slider.value)

        # x = L1 cos(t1) + L2 cos(t1 + t2)
        x_val = L1 * np.cos(t1_val) + L2 * np.cos(t1_val + t2_val)
        y_val = L1 * np.sin(t1_val) + L2 * np.sin(t1_val + t2_val)

    else:
        # --- INVERSE KINEMATICS ---
        x_val = target_x.value
        y_val = target_y.value

        # Law of Cosines
        r = np.sqrt(x_val**2 + y_val**2)

        # Domain check: Can we reach?
        if r > (L1 + L2) or r < abs(L1 - L2) or r < 1e-10:
            valid_ik = False
        else:
            # Calculate Theta 2 (Elbow Down)
            cos_t2 = (x_val**2 + y_val**2 - L1**2 - L2**2) / (2 * L1 * L2)
            cos_t2 = np.clip(cos_t2, -1.0, 1.0)  # Numerical stability
            t2_val = -np.arccos(cos_t2)  # Elbow-down configuration

            # Calculate Theta 1
            k1 = L1 + L2 * np.cos(t2_val)
            k2 = L2 * np.sin(t2_val)
            t1_val = np.arctan2(y_val, x_val) - np.arctan2(k2, k1)

            # Calculate Alternative Solution (Elbow Up)
            t2_alt = np.arccos(cos_t2)  # Positive angle
            k1_alt = L1 + L2 * np.cos(t2_alt)
            k2_alt = L2 * np.sin(t2_alt)
            t1_alt = np.arctan2(y_val, x_val) - np.arctan2(k2_alt, k1_alt)

    return L1, L2, is_ik, t1_alt, t1_val, t2_alt, t2_val, valid_ik, x_val, y_val


@app.cell
def __(
    L1,
    L2,
    Circle,
    Wedge,
    is_ik,
    mo,
    np,
    plt,
    t1_alt,
    t1_val,
    t2_alt,
    t2_val,
    valid_ik,
    x_val,
    y_val,
):
    # --- VISUALIZATION ---

    fig, ax = plt.subplots(figsize=(6, 6))

    # 1. Draw Workspace (The reachable donut)
    outer_radius = L1 + L2
    inner_radius = abs(L1 - L2)
    workspace = Wedge(
        (0, 0),
        outer_radius,
        0,
        360,
        width=outer_radius - inner_radius,
        facecolor="#e0e0e0",
        alpha=0.5,
        label="Workspace",
    )
    ax.add_patch(workspace)

    # Helper to get joint positions
    def get_joints(th1, th2):
        j1_x = L1 * np.cos(th1)
        j1_y = L1 * np.sin(th1)
        ee_x = j1_x + L2 * np.cos(th1 + th2)
        ee_y = j1_y + L2 * np.sin(th1 + th2)
        return (0, 0), (j1_x, j1_y), (ee_x, ee_y)

    # 2. Draw Main Arm
    color = "#2563eb" if valid_ik else "#ef4444"  # Blue if valid, Red if invalid

    if valid_ik:
        base, j1, ee = get_joints(t1_val, t2_val)

        # Links
        ax.plot(
            [base[0], j1[0]],
            [base[1], j1[1]],
            color=color,
            linewidth=4,
            solid_capstyle="round",
        )
        ax.plot(
            [j1[0], ee[0]],
            [j1[1], ee[1]],
            color=color,
            linewidth=4,
            solid_capstyle="round",
        )

        # Joints
        ax.add_patch(Circle(base, 0.15, color="#333"))
        ax.add_patch(Circle(j1, 0.15, color="white", ec="#333", lw=2))
        ax.add_patch(Circle(ee, 0.15, color=color))

        # 3. Draw Ghost Arm (Alternative Solution for IK)
        if is_ik and t1_alt is not None:
            _, j1_a, ee_a = get_joints(t1_alt, t2_alt)
            ax.plot(
                [base[0], j1_a[0]],
                [base[1], j1_a[1]],
                color="#9ca3af",
                linewidth=2,
                linestyle="--",
            )
            ax.plot(
                [j1_a[0], ee_a[0]],
                [j1_a[1], ee_a[1]],
                color="#9ca3af",
                linewidth=2,
                linestyle="--",
            )
            ax.add_patch(Circle(j1_a, 0.1, color="white", ec="#9ca3af"))

    else:
        # Just draw the target point in red if unreachable
        ax.plot(x_val, y_val, "rx", markersize=10)
        ax.text(
            0,
            0,
            "TARGET UNREACHABLE",
            fontsize=12,
            color="red",
            ha="center",
            va="center",
            fontweight="bold",
            bbox=dict(facecolor="white", alpha=0.8),
        )

    # Formatting
    max_range = L1 + L2 + 0.5
    ax.set_xlim(-max_range, max_range)
    ax.set_ylim(-max_range, max_range)
    ax.set_aspect("equal")
    ax.grid(True, linestyle=":", alpha=0.6)
    ax.axhline(0, color="black", linewidth=0.5)
    ax.axvline(0, color="black", linewidth=0.5)
    ax.set_title(f"End Effector: ({x_val:.2f}, {y_val:.2f})", fontsize=11)

    plot_chart = mo.ui.matplotlib(fig)
    return ax, base, color, ee, fig, get_joints, j1, max_range, plot_chart, workspace


@app.cell
def __(
    L1,
    L2,
    is_ik,
    l1_slider,
    l2_slider,
    mo,
    mode_switch,
    np,
    plot_chart,
    t1_val,
    t2_val,
    target_x,
    target_y,
    theta1_slider,
    theta2_slider,
    valid_ik,
    x_val,
    y_val,
):
    # --- EXPLANATION & LAYOUT ---

    # Math Explanation Block
    if not is_ik:
        math_md = mo.md(
            f"""
            ### 📐 Forward Kinematics Math
            We know $\\theta_1, \\theta_2$ and want $(x, y)$.

            $$
            x = L_1 \\cos(\\theta_1) + L_2 \\cos(\\theta_1 + \\theta_2)
            $$

            $$
            x = {L1} \\cos({np.rad2deg(t1_val):.1f}^\\circ)
            + {L2} \\cos({np.rad2deg(t1_val + t2_val):.1f}^\\circ)
            = \\mathbf{{{x_val:.2f}}}
            $$

            ---

            $$
            y = L_1 \\sin(\\theta_1) + L_2 \\sin(\\theta_1 + \\theta_2)
            $$

            $$
            y = {L1} \\sin({np.rad2deg(t1_val):.1f}^\\circ)
            + {L2} \\sin({np.rad2deg(t1_val + t2_val):.1f}^\\circ)
            = \\mathbf{{{y_val:.2f}}}
            $$
            """
        )
        controls = mo.vstack(
            [
                mo.md("### ⚙️ Configuration"),
                l1_slider,
                l2_slider,
                mo.md("---"),
                mode_switch,
                mo.md("### 🎮 Joint Controls"),
                theta1_slider,
                theta2_slider,
            ]
        )
        explanation = None
    else:
        if valid_ik:
            explanation = f"""
            ### 🧠 Inverse Kinematics Math
            We know $(x={x_val}, y={y_val})$ and want angles.

            **1. Law of Cosines for $\\theta_2$:**
            $$
            \\cos(\\theta_2) = \\frac{{x^2 + y^2 - L_1^2 - L_2^2}}{{2 L_1 L_2}}
            $$
            This gives us **two** valid angles for $\\theta_2$ (elbow up/down),
            shown by the dashed ghost arm.

            **2. Geometry for $\\theta_1$:**
            $$
            \\theta_1 = \\arctan(\\frac{{y}}{{x}}) -
            \\arctan(\\frac{{L_2 \\sin(\\theta_2)}}{{L_1 + L_2 \\cos(\\theta_2)}})
            $$

            Current Solution:
            $\\theta_1 = {np.rad2deg(t1_val):.1f}^\\circ$,
            $\\theta_2 = {np.rad2deg(t2_val):.1f}^\\circ$
            """
        else:
            explanation = """
            ### ⚠️ Out of Reach!
            The target $(x, y)$ is outside the workspace.

            The target lies beyond the reachable area, which is bounded by
            $(L_1 + L_2)$ and $|L_1 - L_2|$. As a result, $\\cos(\\theta_2)$
            falls outside $[-1, 1]$, making arccos undefined.
            """

        math_md = mo.md(explanation)

        controls = mo.vstack(
            [
                mo.md("### ⚙️ Configuration"),
                l1_slider,
                l2_slider,
                mo.md("---"),
                mode_switch,
                mo.md("### 🎯 Target Position"),
                target_x,
                target_y,
            ]
        )

    # Final Layout
    mo.hstack(
        [mo.vstack([controls, math_md], align="start", gap=2), plot_chart],
        justify="start",
        gap=4,
    )
    return controls, explanation, math_md


if __name__ == "__main__":
    app.run()
