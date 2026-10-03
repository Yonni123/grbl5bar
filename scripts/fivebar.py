import math
from dataclasses import dataclass

import matplotlib.pyplot as plt
import numpy as np


# ============================================================
# Robot geometry
# ============================================================

@dataclass
class FiveBar:
    motor_spacing: float
    main_arm: float
    forearm: float
    zero_offset: tuple = (0.0, 0.0)  # (X, Y) in mm


    def fk(self, theta1: float, theta2: float):
        """
        Convert motor angles -> Cartesian position.

        Input:
            theta1, theta2 : motor angles in degrees

        Returns:
            x, y : Cartesian position
        """
        theta1_r = math.radians(theta1)
        theta2_r = math.radians(theta2)

        # This assumes that the left motor is at (0,0) and the right motor is at (d,0)
        d = self.motor_spacing

        # 1. Calculate elbow positions
        xl = self.main_arm * math.cos(theta1_r)      # Left elbow
        yl = self.main_arm * math.sin(theta1_r)
        x2 = d + self.main_arm * math.cos(theta2_r)  # Right elbow
        y2 = self.main_arm * math.sin(theta2_r)

        # 2. Straight-line distance between the two elbows
        dx = x2 - xl
        dy = y2 - yl
        line = math.sqrt(dx**2 + dy**2)
        if line > 2 * self.forearm:    # The other configuration is physically impossible
            raise ValueError("Target position is unreachable.")

        # 2.1 Both main arms must point "away" from each other.
        # So we analyze the direction between M1-E1 and M2-E2.
        left_direction  = (xl, yl)
        right_direction = (x2 - d, y2)
        # TODO: Make sure the angles are valid.

        # 3. Circle-Circle Intersection Geometry
        # If we draw two circles with the raduis 'forearm' around the elbows,
        # They will intersect at two points, those are the pen positions.
        # My robot is constraint in "elbows out" configuration, so one will be eliminated.
        # ||left_elbow - P|| = ||right_elbow - P|| = forearm

        # 3.1 Solve for M (point between the two elbows)
        mx = (xl + x2) / 2
        my = (yl + y2) / 2

        # 3.2 Calculate the distance from M to pen position using pythagorean theorem
        h = math.sqrt(self.forearm**2 - (line / 2)**2)

        # Either give X -h and Y +h or vise versa.
        # My robot is locked into the "elbows out" configuration
        # So I will always give the sign that results in bigger py
        py1 = my + h * (x2 - xl) / line
        py2 = my - h * (x2 - xl) / line
        if py1 > py2:   # We are applying the zero offset to the final result
            px1 = mx - h * (y2 - yl) / line
            return px1 - self.zero_offset[0], py1 - self.zero_offset[1]
        else:
            px2 = mx + h * (y2 - yl) / line
            return px2 - self.zero_offset[0], py2 - self.zero_offset[1]

        # --------------------------------------------------------
        # Draw robot
        # --------------------------------------------------------

    def _solve_single_ik_arm(self, x: float, y: float, motor_x: float):
        """
        Solve for the motor angle given a pen position and motor position.

        Input:
            x, y : Cartesian position of the pen
            motor_x : x position of the motor

        Returns:
            alpha : The absolute baseline angle (in radians) from the motor base 
                    directly to the target pen position.
            beta  : The interior swing angle (in radians) between the main arm 
                    and the baseline. Combined with alpha (alpha +/- beta), 
                    this determines the final elbow-in or elbow-out configuration.
        """

        # 1. calculate alpha, simple atan2 from motor to pen position
        alpha = math.atan2(y, x - motor_x)

        # 2. calculate the length of the line from the motor to the pen position
        line = math.sqrt((x - motor_x)**2 + y**2)

        # 3. Guard checks: Target must be within the maximum physical extension 
        # and outside the minimum physical folding limit.
        max_reach = self.main_arm + self.forearm
        min_reach = abs(self.main_arm - self.forearm)
        
        if line > max_reach or line < min_reach:
            raise ValueError("Target position is physically unreachable.")

        # 4. calculate beta using the law of cosines
        # Here, we have:
        # - side a = main_arm
        # - side b = line
        # - side c = forearm (opposite to angle beta)
        # We want to find angle beta (between main_arm and line)
        cos_beta = (self.main_arm**2 + line**2 - self.forearm**2) / (2 * self.main_arm * line)
        
        # Clamp to avoid floating point precision errors crashing math.acos
        cos_beta = max(-1.0, min(1.0, cos_beta))
        
        beta = math.acos(cos_beta)

        return alpha, beta

    def ik(self, x: float, y: float):
        """
        Convert Cartesian position -> motor angles.

        Input:
            x, y : Cartesian position

        Returns:
            theta1, theta2 : motor angles in degrees
        """
        # 0. Apply zero offset to the target position
        x += self.zero_offset[0]
        y += self.zero_offset[1]

        # 1. Solve the geometry (alpha and beta) for both arms independently
        alpha_L, beta_L = self._solve_single_ik_arm(x, y, 0.0)
        alpha_R, beta_R = self._solve_single_ik_arm(x, y, self.motor_spacing)

        # 2. Combine alpha and beta to enforce an "Elbow-Out" configuration
        # Left elbow bends out to the left (+ beta)
        theta1_rad = alpha_L + beta_L
        
        # Right elbow bends out to the right (- beta)
        theta2_rad = alpha_R - beta_R

        # 3. Convert the resulting radians into degrees
        theta1 = math.degrees(theta1_rad)
        theta2 = math.degrees(theta2_rad)

        return theta1, theta2

    def draw_robot(self, val1, val2, mode="ik"):
        """
        Draw the robot using the logical Cartesian coordinate system.

        The logical origin (0, 0) is defined by self.zero_offset.
        The physical motors are therefore shown at their positions
        relative to that logical origin.

        mode="ik":
            val1, val2 = logical Cartesian X, Y

        mode="fk":
            val1, val2 = motor angles theta1, theta2
        """

        # ---------------------------------------------------------
        # Determine pen position and motor angles
        # ---------------------------------------------------------

        if mode.lower() == "fk":
            theta1, theta2 = val1, val2

            # FK already returns LOGICAL coordinates
            pen_x, pen_y = self.fk(theta1, theta2)

        elif mode.lower() == "ik":
            # Input is already in LOGICAL coordinates
            pen_x, pen_y = val1, val2

            # IK also expects LOGICAL coordinates
            theta1, theta2 = self.ik(pen_x, pen_y)

        else:
            raise ValueError("mode must be either 'ik' or 'fk'")

        # ---------------------------------------------------------
        # Convert logical pen position to physical coordinates
        # for calculating the actual robot geometry
        # ---------------------------------------------------------

        physical_pen_x = pen_x + self.zero_offset[0]
        physical_pen_y = pen_y + self.zero_offset[1]

        # ---------------------------------------------------------
        # Physical motor positions
        # ---------------------------------------------------------

        left_motor_physical = (0.0, 0.0)
        right_motor_physical = (self.motor_spacing, 0.0)

        # ---------------------------------------------------------
        # Calculate elbow positions in PHYSICAL coordinates
        # ---------------------------------------------------------

        theta1_rad = np.radians(theta1)
        theta2_rad = np.radians(theta2)

        left_elbow_physical = (
            self.main_arm * np.cos(theta1_rad),
            self.main_arm * np.sin(theta1_rad)
        )

        right_elbow_physical = (
            self.motor_spacing + self.main_arm * np.cos(theta2_rad),
            self.main_arm * np.sin(theta2_rad)
        )

        pen_physical = (
            physical_pen_x,
            physical_pen_y
        )

        # ---------------------------------------------------------
        # Convert all physical points to LOGICAL/PLOT coordinates
        #
        # This makes the plot origin coincide with the physical
        # workspace origin.
        # ---------------------------------------------------------

        def to_plot(point):
            x, y = point
            return (
                x - self.zero_offset[0],
                y - self.zero_offset[1]
            )

        left_motor = to_plot(left_motor_physical)
        right_motor = to_plot(right_motor_physical)
        left_elbow = to_plot(left_elbow_physical)
        right_elbow = to_plot(right_elbow_physical)
        pen = to_plot(pen_physical)

        # ---------------------------------------------------------
        # Plot
        # ---------------------------------------------------------

        fig, ax = plt.subplots()

        # Left motor -> left elbow
        ax.plot(
            [left_motor[0], left_elbow[0]],
            [left_motor[1], left_elbow[1]],
            'o-',
            linewidth=3,
            markersize=8,
            color="tab:blue"
        )

        # Left elbow -> pen
        ax.plot(
            [left_elbow[0], pen[0]],
            [left_elbow[1], pen[1]],
            'o-',
            linewidth=3,
            markersize=8,
            color="tab:orange"
        )

        # Right motor -> right elbow
        ax.plot(
            [right_motor[0], right_elbow[0]],
            [right_motor[1], right_elbow[1]],
            'o-',
            linewidth=3,
            markersize=8,
            color="tab:blue"
        )

        # Right elbow -> pen
        ax.plot(
            [right_elbow[0], pen[0]],
            [right_elbow[1], pen[1]],
            'o-',
            linewidth=3,
            markersize=8,
            color="tab:orange"
        )

        # ---------------------------------------------------------
        # Workspace
        # ---------------------------------------------------------

        ax.set_aspect("equal", adjustable="box")

        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")

        ax.set_title(
            f"Five-Bar Robot ({mode.upper()})\n"
            f"Pen = ({pen_x:.2f}, {pen_y:.2f}) mm"
        )

        ax.grid(True)

        plt.show()



if __name__ == "__main__":

    # Initialize the robot with physical properties
    robot = FiveBar(
        motor_spacing=80.0,
        main_arm=120.0,
        forearm=155.0,
        zero_offset=(-65.000, 65.765)  # Origo (bottom-left corner) position in the robot's cartesian coordinate system
    )

    # Forward Kinematics (FK)
    theta1 = 227.44    # This is the origo position (0,0)
    theta2 = 89.904

    calc_x, calc_y = robot.fk(theta1, theta2)

    print("\n--- Forward Kinematics (FK) Test ---")
    print(f"Input Angles -> theta1: {theta1:.3f}°, theta2: {theta2:.3f}°")
    print(f"Output Pen   -> x: {calc_x:.3f} mm, y: {calc_y:.3f} mm\n")

    robot.draw_robot(mode="fk", val1=theta1, val2=theta2)

    # Inverse Kinematics (IK)
    x = 0
    y = 0

    calc_theta1, calc_theta2 = robot.ik(x, y)

    print("--- Inverse Kinematics (IK) Test ---")
    print(f"Input Pen    -> x: {x:.3f} mm, y: {y:.3f} mm")
    print(f"Output Angles -> theta1: {calc_theta1:.3f}°, theta2: {calc_theta2:.3f}°\n")

    robot.draw_robot(mode="ik", val1=x, val2=y)
