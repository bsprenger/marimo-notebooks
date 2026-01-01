"""Interactive notebook demonstrating Euler Angles vs. Quaternions in robotics."""

import marimo

__generated_with = "0.1.0"
app = marimo.App(width="full")


@app.cell
def __():
    import marimo as mo
    import matplotlib.pyplot as plt
    import numpy as np
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    # We use scipy for robust rotation math
    from scipy.spatial.transform import Rotation as R

    return Poly3DCollection, R, mo, np, plt


@app.cell
def __(mo):
    mo.md(
        """
        # 🔄 The Language of 3D: Euler Angles vs. Quaternions

        In Robotics and RL, how we represent "orientation" matters.

        1.  **Euler Angles (Roll, Pitch, Yaw):** Intuitive for humans, but suffer from **Gimbal Lock**.
        2.  **Quaternions ($w, x, y, z$):** Intuitive for computers, singularity-free, used in nearly all SOTA RL observation spaces.

        Use this notebook to break the Euler representation and understand why we use Quaternions.
        """
    )
    return ()


@app.cell
def __(mo):
    # --- CONTROLS ---

    mo.md("### 🎮 Control Panel")

    # 1. Euler Controls
    roll = mo.ui.slider(-180, 180, step=1, value=0, label="Roll ($x$)")
    pitch = mo.ui.slider(-90, 90, step=1, value=0, label="Pitch ($y$)")
    yaw = mo.ui.slider(-180, 180, step=1, value=0, label="Yaw ($z$)")

    # 2. Modes
    lock_demo = mo.ui.checkbox(
        label="🔒 **Activate Gimbal Lock Mode** (Forces Pitch to 90°)"
    )

    mo.hstack(
        [
            mo.vstack([roll, pitch, yaw, lock_demo]),
            mo.md(
                """
            **Try this to find Gimbal Lock:**
            1. Set **Pitch** to exactly **90°**.
            2. Now move **Roll**. Notice what happens.
            3. Now move **Yaw**. Notice what happens.

            *They perform the exact same rotation!* You have lost one degree of freedom.
            """
            ).callout(kind="warning"),
        ],
        gap=4,
    )
    return lock_demo, pitch, roll, yaw


@app.cell
def __(lock_demo, np, pitch, roll, yaw):
    # --- STATE CALCULATION ---

    # Get values
    r_val = roll.value
    p_val = 90 if lock_demo.value else pitch.value
    y_val = yaw.value

    # Create Rotation Object (ZYX convention is standard for mobile robots)
    # Note: 'xyz' in scipy refers to extrinsic (static frame), 'XYZ' is intrinsic (moving frame)
    # Robotics usually uses Intrinsic rotations.
    rot = R.from_euler("xyz", [r_val, p_val, y_val], degrees=True)

    # Extract Quaternion
    quat = rot.as_quat()  # [x, y, z, w] format in scipy

    # Extract Rotation Matrix
    matrix = rot.as_matrix()

    return matrix, p_val, quat, r_val, rot, y_val


@app.cell
def __(Poly3DCollection, matrix, mo, np, p_val, plt, quat, r_val, y_val):
    # --- VISUALIZATION ENGINE ---

    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(111, projection="3d")

    # 1. Define a 3D Object (An asymmetric Airplane/Dart to see orientation clearly)
    # Vertices of a simple dart
    v = np.array(
        [
            [1.5, 0.0, 0.0],  # Nose
            [-1.0, 0.5, 0.0],  # Left Wing
            [-1.0, -0.5, 0.0],  # Right Wing
            [-1.0, 0.0, 0.5],  # Tail
            [-1.0, 0.0, 0.0],  # Rear Center
            [1.5, 0.0, 0.0],  # Close loop
        ]
    )

    # Rotate the vertices
    v_rotated = (matrix @ v.T).T

    # 2. Draw the Object
    # Create faces for plotting
    faces = [
        [v_rotated[0], v_rotated[1], v_rotated[4]],  # Left Wing
        [v_rotated[0], v_rotated[2], v_rotated[4]],  # Right Wing
        [v_rotated[0], v_rotated[3], v_rotated[4]],  # Tail
        [v_rotated[1], v_rotated[2], v_rotated[4]],  # Back
        [v_rotated[1], v_rotated[3], v_rotated[2]],  # Back 2
    ]

    poly = Poly3DCollection(faces, alpha=0.7, edgecolor="k")
    poly.set_facecolor(
        ["#60a5fa", "#60a5fa", "#ef4444", "#9ca3af", "#9ca3af"]
    )  # Blue wings, Red tail
    ax.add_collection3d(poly)

    # 3. Draw Local Axes (The Gimbal Rings)
    np.array([0, 0, 0])
    # X Axis (Red) - Forward
    x_axis = matrix @ np.array([2, 0, 0])
    ax.quiver(
        0,
        0,
        0,
        x_axis[0],
        x_axis[1],
        x_axis[2],
        color="r",
        arrow_length_ratio=0.1,
        linewidth=2,
    )
    ax.text(x_axis[0], x_axis[1], x_axis[2], "X", color="r")

    # Y Axis (Green) - Left
    y_axis = matrix @ np.array([0, 2, 0])
    ax.quiver(
        0,
        0,
        0,
        y_axis[0],
        y_axis[1],
        y_axis[2],
        color="g",
        arrow_length_ratio=0.1,
        linewidth=2,
    )
    ax.text(y_axis[0], y_axis[1], y_axis[2], "Y", color="g")

    # Z Axis (Blue) - Up
    z_axis = matrix @ np.array([0, 0, 2])
    ax.quiver(
        0,
        0,
        0,
        z_axis[0],
        z_axis[1],
        z_axis[2],
        color="b",
        arrow_length_ratio=0.1,
        linewidth=2,
    )
    ax.text(z_axis[0], z_axis[1], z_axis[2], "Z", color="b")

    # Plot formatting
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.set_zlim(-2, 2)
    ax.set_xlabel("Global X")
    ax.set_ylabel("Global Y")
    ax.set_zlabel("Global Z")
    ax.set_title(f"Euler: [{r_val:.0f}°, {p_val:.0f}°, {y_val:.0f}°]", fontsize=14)

    # --- MATH DISPLAY ---

    # Color code if Gimbal Lock is happening (Pitch near 90)
    is_locked = abs(p_val) == 90
    status_color = "#ef4444" if is_locked else "#22c55e"
    status_text = "⚠️ GIMBAL LOCK ACTIVE" if is_locked else "✅ DOF: 3 (Safe)"

    math_display = mo.md(
        rf"""
        ### 🧮 Representation

        **1. Euler Angles (The Input)**
        Intuitive, but dangerous.
        $$ [\phi, \theta, \psi] = [{r_val}^\circ, {p_val}^\circ, {y_val}^\circ] $$

        <div style="padding: 10px; background-color: {status_color}20; border-left: 5px solid {status_color};">
        <b>Status:</b> {status_text}
        </div>

        ---

        **2. Quaternion (The RL Observation)**
        This is what your policy network actually sees.
        $$ q = [x={quat[0]:.3f}, \; y={quat[1]:.3f}, \; z={quat[2]:.3f}, \; w={quat[3]:.3f}] $$

        **Why Quaternions for RL?**
        * **No Singularities:** You can represent any orientation continuously.
        * **Normalized:** The constraint $x^2+y^2+z^2+w^2 = 1$ is easy for neural networks to maintain (mostly).
        * **Double Cover:** Notice that $q$ and $-q$ represent the same rotation.
        """
    )

    plot_obj = mo.ui.matplotlib(fig)
    return is_locked, math_display, plot_obj, status_color, status_text


@app.cell
def __(is_locked, math_display, mo, plot_obj):
    explanation = (
        mo.md(
            """
        ### 🎓 What is happening?

        **The Gimbal Lock Problem:**
        Euler angles are applied sequentially: First Roll, *then* Pitch, *then* Yaw.

        When you pitch up by **90 degrees**:
        1. The robot's nose points straight up.
        2. The "Roll" axis (X) and the "Yaw" axis (Z) become **parallel**.
        3. Rotating around X becomes mathematically identical to rotating around Z.

        **The Consequence for RL:**
        If your RL agent uses Euler angles for control, and it looks straight up,
        gradient descent creates massive instability because two actions do the exact same thing.
        The "Jacobian" loses rank.
        """
        )
        if is_locked
        else mo.md(
            """
        ### ℹ️ How to break it

        Right now, the axes are distinct.
        * **Red** is Forward.
        * **Green** is Left.
        * **Blue** is Up.

        Try setting **Pitch to 90**. Watch the Red axis align with the World Z axis.
        """
        )
    )

    mo.hstack([plot_obj, mo.vstack([math_display, explanation])], gap=4)
    return (explanation,)


if __name__ == "__main__":
    app.run()
