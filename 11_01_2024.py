from pathlib import Path
"""
code = r
6DOF fixed-wing aircraft simulation
English-commented rewrite of the original script by hamdihunturk.

Important:
- This is a readability/translation rewrite, not a validated flight model.
- The original script contains apparent modelling and implementation issues.
  They are intentionally flagged in comments rather than silently "corrected".
- Angles are expected to be in radians unless stated otherwise.
"""

import numpy as np
import matplotlib.pyplot as plt


#Standard-atmosphere approximation


def atmosphere_model(altitude_m):
    """
    Estimate air temperature, pressure and density at an altitude.

    Args:
        altitude_m: Altitude above sea level, in metres.

    Returns:
        temperature_K, pressure_Pa, density_kg_m3
    """
    R_air = 287.05       # Specific gas constant for dry air [J/(kg K)]
    pressure_sl = 101325.0  # Standard sea-level pressure [Pa]
    temperature_sl = 288.15  # Standard sea-level temperature [K]
    gravity = 9.80665    # Gravitational acceleration [m/s^2]
    lapse_rate = 6.5 / 1000.0  # Temperature lapse rate [K/m]
    density_sl = 1.225   # Standard sea-level density [kg/m^3]

    temperature = temperature_sl - lapse_rate * altitude_m
    pressure = pressure_sl * (
        1.0 - lapse_rate * altitude_m / temperature_sl
    ) ** (gravity / (R_air * lapse_rate))
    density = density_sl * (pressure / pressure_sl) * (
        temperature_sl / temperature
    )

    return temperature, pressure, density


# Initial atmosphere calculation at 500 m.
altitude_initial = 500.0
temperature, pressure, air_density = atmosphere_model(altitude_initial)

print(f"Altitude: {altitude_initial:.1f} m")
print(f"Temperature: {temperature:.2f} K")
print(f"Pressure: {pressure:.2f} Pa")
print(f"Air density: {air_density:.4f} kg/m^3")


#Control inputs and aircraft / propulsion parameters

throttle = 0.5  # Throttle command, nominally from 0 to 1
elevator_command_deg = 0.0
aileron_command_deg = 2.0
rudder_command_deg = 0.0

# Propeller and motor parameters (values retained from the source).
propeller_area = 0.2027  # The source labels this as propeller diameter [m];
                         # check the intended units/meaning before using it.
propeller_coefficient = 1.0
thrust_coefficient = 0.0
torque_coefficient = 0.0
motor_speed_constant = 80.0
motor_efficiency = 0.9

wing_area = 0.55       # Wing planform area [m^2]
wing_span = 2.8956     # Wing span [m]
mean_chord = 0.18994   # Mean aerodynamic chord [m]

# Control-surface deflections used in the aerodynamic model.
elevator_deflection = 0.0
aileron_deflection = 0.0
rudder_deflection = 0.0

motor_thrust = 1.0
motor_torque = 1.0


# Six-degree-of-freedom equations of motion

def equations_of_motion(state, time, forces_moments, aircraft_parameters):
    """
    Compute the time derivatives of a 12-element aircraft state vector.

    State order:
        [u, v, w, p, q, r, phi, theta, psi, x, y, z]

    u, v, w       Body-axis translational velocities [m/s]
    p, q, r       Body-axis angular velocities [rad/s]
    phi, theta,
    psi           Roll, pitch and yaw Euler angles [rad]
    x, y, z       Position coordinates [m]

    forces_moments is expected in the order [Fx, Fz, Fy, L, M, N] in the
    original script. This unusual order is preserved here for compatibility.
    Confirm that the force order matches the equations before using this model.

    time is accepted for numerical-integrator compatibility but is not used.
    """
    u, v, w, p, q, r, phi, theta, psi, x, y, z = state

    mass = aircraft_parameters["mass"]
    Jxx = aircraft_parameters["Jxx"]
    Jyy = aircraft_parameters["Jyy"]
    Jzz = aircraft_parameters["Jzz"]
    Jxz = aircraft_parameters["Jxz"]

    # Inertia-related Gamma coefficients used by the rotational equations.
    Gamma = Jxx * Jzz - Jxz**2
    Gamma1 = Jxz * (Jxx - Jyy + Jzz) / Gamma
    Gamma2 = (Jzz * (Jzz - Jyy) + Jxz**2) / Gamma
    Gamma3 = Jzz / Gamma
    Gamma4 = Jxz / Gamma
    Gamma5 = (Jzz - Jxx) / Jyy
    Gamma6 = Jxz / Jyy
    Gamma7 = ((Jxx - Jyy) * Jxx + Jxz**2) / Gamma
    Gamma8 = Jxx / Gamma

    # Preserve the force/moment ordering used by the original script.
    Fx, Fz, Fy, roll_moment, pitch_moment, yaw_moment = forces_moments

    # Rotational equations: angular accelerations [rad/s^2].
    dot_p = (
        Gamma1 * p * q - Gamma2 * q * r
        + Gamma3 * roll_moment + Gamma4 * yaw_moment
    )
    dot_q = (
        Gamma5 * p * r - Gamma6 * (p**2 - r**2)
        + pitch_moment / Jyy
    )
    dot_r = (
        Gamma7 * p * q - Gamma1 * q * r
        + Gamma4 * roll_moment + Gamma8 * yaw_moment
    )

    # Translational equations in body axes.
    # The force signs/order below are preserved from the source and should be
    # checked against the chosen body-axis convention (commonly x forward,
    # y right, z down).
    dot_u = r * v - q * w - Fx / mass
    dot_v = p * w - r * u + Fy / mass
    dot_w = q * u - p * v + Fz / mass

    # Convert body angular velocities p, q, r into Euler-angle rates.
    dot_phi = p + q * np.sin(phi) * np.tan(theta) + r * np.cos(phi) * np.tan(theta)
    dot_theta = q * np.cos(phi) - r * np.sin(phi)
    dot_psi = (
        q * np.sin(phi) / np.cos(theta)
        + r * np.cos(phi) / np.cos(theta)
    )

    # Transform body-axis linear velocity into position rates in the reference
    # frame using the roll-pitch-yaw rotation matrix.
    sin_phi, cos_phi = np.sin(phi), np.cos(phi)
    sin_theta, cos_theta = np.sin(theta), np.cos(theta)
    sin_psi, cos_psi = np.sin(psi), np.cos(psi)

    dot_x = (
        cos_theta * cos_psi * u
        + (sin_phi * sin_theta * cos_psi - cos_phi * sin_psi) * v
        + (cos_phi * sin_theta * cos_psi + sin_phi * sin_psi) * w
    )
    dot_y = (
        cos_theta * sin_psi * u
        + (sin_phi * sin_theta * sin_psi + cos_phi * cos_psi) * v
        + (cos_phi * sin_theta * sin_psi - sin_phi * cos_psi) * w
    )
    dot_z = (
        -sin_theta * u
        + sin_phi * cos_theta * v
        + cos_phi * cos_theta * w
    )

    return [
        dot_u, dot_v, dot_w,
        dot_p, dot_q, dot_r,
        dot_phi, dot_theta, dot_psi,
        dot_x, dot_y, dot_z,
    ]


#Aerodynamic forces and moments

def aerodynamic_forces_moments(
    airspeed, density, wing_area, span, chord,
    p, q, r, phi, theta, alpha, beta,
    elevator, aileron, rudder=0.0,
):
    """
    Estimate aerodynamic forces and moments using coefficient equations.

    Args:
        airspeed: Airspeed [m/s].
        density: Air density [kg/m^3].
        wing_area: Wing area [m^2].
        span: Wing span [m].
        chord: Mean aerodynamic chord [m].
        p, q, r: Body angular rates [rad/s].
        phi, theta: Roll and pitch angles [rad].
        alpha: Angle of attack [rad].
        beta: Sideslip angle [rad].
        elevator, aileron, rudder: Control-surface deflections. The original
            code appears to use radians in the coefficient equations.

    Returns:
        Fx, Fy, Fz, roll_moment, pitch_moment, yaw_moment

    Note: Several expressions below are preserved from the original source,
    including expressions that appear dimensionally or mathematically suspect.
    Validate them against a trusted aircraft model before using the simulation.
    """
    dynamic_pressure = 0.5 * density * airspeed**2
    gravity = 9.81
    mass_for_force_model = 13.5

    # Lift coefficient model.
    CL_q = 0.0
    CL_elevator = -0.36
    CL_0 = 0.28
    CL_alpha = 3.45
    CL_base = CL_0 + CL_alpha * alpha
    CL = CL_base + (CL_q * chord * q / (2.0 * airspeed)) + CL_elevator * elevator

    # Drag coefficient model.
    CD_q = 0.0
    CD_elevator = 0.0
    CD_0 = 0.03
    CD_alpha = 0.30
    CD_base = CD_0 + CD_alpha * alpha
    CD = CD_base + (CD_q * chord * q / (2.0 * airspeed)) + CD_elevator * elevator

    # Side-force coefficient model.
    CY_0 = 0.0
    CY_beta = -0.98
    CY_p = 0.0
    CY_r = 0.0
    CY_aileron = 0.0
    CY_rudder = -0.17
    CY = (
        CY_0
        + CY_beta * beta
        + CY_p * span * p / (2.0 * airspeed)
        + CY_r * span * r / (2.0 * airspeed)
        + CY_aileron * aileron
        + CY_rudder * rudder
    )

    # Lift, drag and side force magnitudes.
    lift = dynamic_pressure * wing_area * CL
    drag = dynamic_pressure * wing_area * CD
    side_force = dynamic_pressure * wing_area * CY

    # Resolve aerodynamic coefficients into body-axis force coefficients.
    CX_alpha = -CD_base * np.cos(alpha) + CL_base * np.sin(alpha)
    CX_q = -CD_q * np.cos(alpha) + CL_q * np.sin(alpha)
    CX_elevator = -CD_elevator * np.cos(alpha) + CL_elevator * np.sin(alpha)

    CZ_alpha = -CD_base * np.sin(alpha) - CL_base * np.cos(alpha)
    CZ_q = -CD_q * np.sin(alpha) - CL_q * np.cos(alpha)
    CZ_elevator = -CD_elevator * np.sin(alpha) - CL_elevator * np.cos(alpha)

    # Body-axis forces. The original script's gravity terms and signs are
    # retained for comparison, but should be checked before relying on results.
    Fx = (
        -gravity * np.sin(theta)
        + dynamic_pressure * wing_area * (
            CX_alpha
            + CX_q * chord * q / (2.0 * airspeed)
            + CX_elevator * elevator
        )
        + motor_thrust
    )
    Fy = mass_for_force_model * gravity * np.cos(theta) * np.sin(phi) + side_force
    Fz = (
        mass_for_force_model * gravity * np.cos(theta) * np.cos(phi)
        + dynamic_pressure * wing_area * (
            CZ_alpha
            + CZ_q * chord * q / (2.0 * airspeed)
            + CZ_elevator * elevator
        )
    )

    # Rolling-moment coefficients.
    Cl_0 = 0.0
    Cl_beta = -0.12
    Cl_p = -0.26
    Cl_r = 0.14
    Cl_aileron = 0.08
    Cl_rudder = 0.105

    # Pitching-moment coefficients.
    Cm_0 = -0.02338
    Cm_alpha = -0.38
    Cm_q = -3.6
    Cm_elevator = -0.5

    # Yawing-moment coefficients.
    Cn_0 = 0.0
    Cn_beta = 0.25
    Cn_p = 0.022
    Cn_r = -0.35
    Cn_aileron = 0.06
    Cn_rudder = -0.032

    # The original script's final terms include "+ Cl_rudder + rudder" and
    # "+ Cn_rudder + rudder" rather than multiplying by rudder. Those forms
    # are retained here to make the rewrite traceable to the source.
    Cl = (
        Cl_0 + Cl_beta * beta
        + Cl_p * span * p / (2.0 * airspeed)
        + Cl_r * span * r / (2.0 * airspeed)
        + Cl_aileron * aileron + Cl_rudder + rudder
    )
    Cm = (
        Cm_0 + Cm_alpha * alpha
        + Cm_q * chord * q / (2.0 * airspeed)
        + Cm_elevator * elevator
    )
    Cn = (
        Cn_0 + Cn_beta * beta
        + Cn_p * span * p / (2.0 * airspeed)
        + Cn_r * span * r / (2.0 * airspeed)
        + Cn_aileron * aileron + Cn_rudder + rudder
    )

    roll_moment = dynamic_pressure * wing_area * span * Cl + motor_torque
    pitch_moment = dynamic_pressure * wing_area * chord * Cm
    yaw_moment = dynamic_pressure * wing_area * span * Cn

    return Fx, Fy, Fz, roll_moment, pitch_moment, yaw_moment


# Simple propeller / motor model


class MotorModel:
    """Simple propeller thrust and torque approximation from the source."""

    def __init__(
        self, density, propeller_area, propeller_coefficient,
        motor_speed_constant, thrust_coefficient, torque_coefficient,
        efficiency,
    ):
        self.density = density
        self.propeller_area = propeller_area
        self.propeller_coefficient = propeller_coefficient
        self.thrust_coefficient = thrust_coefficient
        self.torque_coefficient = torque_coefficient
        self.motor_speed_constant = motor_speed_constant
        self.efficiency = efficiency

    def calculate_thrust_torque(self, airspeed, throttle_command):
        """Return estimated propeller thrust [N] and torque [N m]."""
        thrust = (
            0.5 * self.density * self.propeller_area * self.propeller_coefficient
            * ((self.motor_speed_constant * throttle_command)**2 - airspeed**2)
        )
        torque = -self.thrust_coefficient * (
            self.torque_coefficient * throttle_command
        )**2

        thrust *= self.efficiency
        torque *= self.efficiency
        return thrust, torque


# Angle-of-attack and sideslip calculations


def calculate_alpha_beta(w, u, airspeed):
    """
    Calculate angle of attack alpha and sideslip beta.

    The source computes beta as arcsin(u / airspeed), which is unusual:
    conventional body-axis definitions generally use beta = arcsin(v / Va).
    This source expression is preserved but should be checked.
    """
    alpha = np.arctan(w / u)
    beta = np.arcsin(u / airspeed)
    return alpha, beta


#  Control-surface input schedules


def elevator_input(time_s):
    """Apply the source's elevator input between 10.0 and 10.2 seconds."""
    elevator, aileron = 0.0, 0.0
    if 10.0 <= time_s <= 10.2:
        # Original comments describe symmetric movement of the two surfaces.
        right_surface = elevator_command_deg
        left_surface = elevator_command_deg
        elevator = right_surface + left_surface
        aileron = -right_surface + left_surface
    return elevator, aileron


def aileron_input(time_s):
    """Apply the source's aileron input between 10.0 and 10.2 seconds."""
    elevator, aileron = 0.0, 0.0
    if 10.0 <= time_s <= 10.2:
        right_surface = aileron_command_deg
        left_surface = -aileron_command_deg
        elevator = right_surface + left_surface
        aileron = -right_surface + left_surface
    return elevator, aileron


# Simulation setup


simulation_duration = 30.0  # [s]
time_step = 0.1              # Nominal integration time step [s]
number_of_steps = int(simulation_duration / time_step)
time_values = np.linspace(0.0, simulation_duration, number_of_steps)

# State vector:
# [u, v, w, p, q, r, roll, pitch, yaw, x, y, z]
# The original comment says the aircraft starts at 500 m altitude. Its z-axis
# sign convention must be confirmed: with NED axes, positive z is down.
state = [0.1, 0.1, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 500.0]

aircraft_parameters = {
    "mass": 13.5,  # [kg]
    "Jxx": 0.8244, # Roll-axis moment of inertia [kg m^2]
    "Jyy": 1.135,  # Pitch-axis moment of inertia [kg m^2]
    "Jzz": 1.759,  # Yaw-axis moment of inertia [kg m^2]
    "Jxz": 0.1204, # Cross-product of inertia [kg m^2]
}

altitude_history = []
airspeed_history = []
pitch_history = []
angle_of_attack_history = []
motor_thrust_history = []
motor_torque_history = []
roll_history = []
yaw_history = []

alpha = 0.4712  # Initial angle of attack [rad]
beta = 0.01      # Initial sideslip angle [rad]


#  Main simulation loop

for i in range(number_of_steps):
    current_time = time_step * i

    if airspeed_history:
        airspeed = airspeed_history[-1]
    else:
        airspeed = 50.0  # Initial airspeed [m/s]

    motor = MotorModel(
        air_density, propeller_area, propeller_coefficient,
        motor_speed_constant, thrust_coefficient, torque_coefficient,
        motor_efficiency,
    )
    motor_thrust, motor_torque = motor.calculate_thrust_torque(
        airspeed, throttle
    )

    # The source calls the aileron schedule; the elevator schedule is commented out.
    elevator_deflection, aileron_deflection = aileron_input(current_time)

    alpha, beta = calculate_alpha_beta(state[2], state[0], airspeed)

    # The source function returns forces in the order Fx, Fy, Fz, L, M, N.
    Fx, Fy, Fz, L, M, N = aerodynamic_forces_moments(
        airspeed, air_density, wing_area, wing_span, mean_chord,
        state[3], state[4], state[5], state[6], state[7],
        alpha, beta, elevator_deflection, aileron_deflection,
        rudder_deflection,
    )

    # The original equations_of_motion expects [Fx, Fz, Fy, L, M, N].
    forces_moments = [Fx, Fz, Fy, L, M, N]

    state_derivative = equations_of_motion(
        state, current_time, forces_moments, aircraft_parameters
    )

    # Preserve the source's update multiplier. It is 0.002 rather than the
    # declared time_step (0.1 s), so the simulated time scaling is inconsistent.
    integration_multiplier = 0.002
    state = [
        state[j] + state_derivative[j] * integration_multiplier
        for j in range(len(state_derivative))
    ]

    altitude_history.append(state[11])
    airspeed_history.append(np.sqrt(state[0]**2 + state[1]**2 + state[2]**2))
    pitch_history.append(state[7])
    roll_history.append(state[6])
    yaw_history.append(state[8])
    angle_of_attack_history.append(np.arctan2(state[2], state[0]))
    motor_thrust_history.append(motor_thrust)
    motor_torque_history.append(motor_torque)


#  Plot simulation results


plt.figure(figsize=(12, 8))

plt.subplot(3, 3, 1)
plt.plot(time_values, altitude_history, label="Altitude coordinate (m)")
plt.xlabel("Time (s)")
plt.ylabel("Altitude coordinate (m)")
plt.title("Altitude coordinate vs time")
plt.legend()

plt.subplot(3, 3, 2)
plt.plot(time_values, airspeed_history, label="Airspeed (m/s)")
plt.xlabel("Time (s)")
plt.ylabel("Airspeed (m/s)")
plt.title("Airspeed vs time")
plt.legend()

plt.subplot(3, 3, 3)
plt.plot(time_values, pitch_history, label="Pitch angle (rad)")
plt.xlabel("Time (s)")
plt.ylabel("Pitch angle (rad)")
plt.title("Pitch angle vs time")
plt.legend()

plt.subplot(3, 3, 4)
plt.plot(time_values, angle_of_attack_history, label="Angle of attack (rad)")
plt.xlabel("Time (s)")
plt.ylabel("Angle of attack (rad)")
plt.title("Angle of attack vs time")
plt.legend()

plt.subplot(3, 3, 5)
plt.plot(time_values, motor_thrust_history, label="Motor thrust")
plt.xlabel("Time (s)")
plt.ylabel("Thrust (N, model-dependent)")
plt.title("Motor thrust vs time")
plt.legend()

plt.subplot(3, 3, 6)
plt.plot(time_values, motor_torque_history, label="Motor torque")
plt.xlabel("Time (s)")
plt.ylabel("Torque (N m, model-dependent)")
plt.title("Motor torque vs time")
plt.legend()

plt.subplot(3, 3, 7)
plt.plot(time_values, roll_history, label="Roll angle (rad)")
plt.xlabel("Time (s)")
plt.ylabel("Roll angle (rad)")
plt.title("Roll angle vs time")
plt.legend()

plt.subplot(3, 3, 8)
plt.plot(time_values, yaw_history, label="Yaw angle (rad)")
plt.xlabel("Time (s)")
plt.ylabel("Yaw angle (rad)")
plt.title("Yaw angle vs time")
plt.legend()

plt.tight_layout()
plt.show()


path = Path("/mnt/data/6dof_aircraft_english_rewrite.py")
path.write_text(code, encoding="utf-8")
print(f"Created {path} ({len(code.splitlines())} lines).")
