"""Soil Sampling Controller with Corrected Depth Target and Trigger."""

from controller import Robot
import math

if __name__ == '__main__':
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    max_speed = 6.28

    # Initialize wheel motors
    motor_fl = robot.getDevice('motor_fl')
    motor_fl.setPosition(float('inf'))
    motor_fl.setVelocity(0.0)

    motor_fr = robot.getDevice('motor_fr')
    motor_fr.setPosition(float('inf'))
    motor_fr.setVelocity(0.0)

    motor_rr = robot.getDevice('motor_rr')
    motor_rr.setPosition(float('inf'))
    motor_rr.setVelocity(0.0)

    motor_rl = robot.getDevice('motor_rl')
    motor_rl.setPosition(float('inf'))
    motor_rl.setVelocity(0.0)

    # Initialise positional wheel sensors
    front_left_sensor = robot.getDevice('ps_fl')
    front_left_sensor.enable(timestep)

    front_right_sensor = robot.getDevice('ps_fr')
    front_right_sensor.enable(timestep)

    rear_left_sensor = robot.getDevice('ps_rl')
    rear_left_sensor.enable(timestep)

    rear_right_sensor = robot.getDevice('ps_rr')
    rear_right_sensor.enable(timestep)

    # Initialize Camera, Force Sensor, Actuator Position Sensor
    camera = robot.getDevice('camera')
    camera.enable(timestep)

    force_sensor = robot.getDevice('force_sensor')
    force_sensor.enable(timestep)

    actuator_ps = robot.getDevice('actuator_ps')
    actuator_ps.enable(timestep)

    # Initialize Linear Actuator & Navigation Sensors
    Actuator_Device = robot.getDevice('linear_actuator')
    Actuator_Device.setVelocity(0.0)

    ds_front = robot.getDevice('ds_front')
    ds_front.enable(timestep)

    imu = robot.getDevice('inertial_unit')
    imu.enable(timestep)

    # Constants & Parameters
    PITCH = 0.002       # Lead screw pitch (meters per rev: 2mm)
    EFFICIENCY = 0.85   # Mechanical efficiency of lead screw (85%)
    K_T = 0.05          # Motor torque constant (Nm / Amp)
    I_NO_LOAD = 0.15    # No-load current draw (Amps)
    R_SENSE = 1000      # BTS7960 IS-pin sense resistor (Ohms)
    K_ILIS = 8500       # BTS7960 current ratio parameter

    FORCE_THRESHOLD = 35.0

    # Compute encoder units
    wheel_radius = 0.025
    wheel_circumference = 2 * 3.14 * wheel_radius
    encoder_unit = wheel_circumference / 6.28

    motors = {
        'fl': motor_fl,
        'fr': motor_fr,
        'rl': motor_rl,
        'rr': motor_rr,
    }

    # State machine setup
    state = "DRIVE"  # DRIVE, EXTEND, SAMPLE_HOLD, RETRACT
    phase_start_time = 0.0
    last_sample_distance = 0.0

    startTime = robot.getTime()

    while robot.step(timestep) != -1:
        currentTime = robot.getTime() - startTime

        # Read Positional Sensors & Distance
        fl_val = front_left_sensor.getValue()
        current_absolute_distance = fl_val * encoder_unit
        distance_traveled = current_absolute_distance - last_sample_distance

        # BTS7960 and Soil Force Calculations
        force_vector = force_sensor.getValues() 
        f_soil = abs(force_vector[2])
        depth = abs(actuator_ps.getValue())    
        torque = (f_soil * PITCH) / (2.0 * 3.14159 * EFFICIENCY)
        i_motor = (torque / K_T) + I_NO_LOAD if f_soil > 0 else I_NO_LOAD
        v_sense = (i_motor / K_ILIS) * R_SENSE

        obstacle_distance = ds_front.getValue()
        roll, pitch, yaw = imu.getRollPitchYaw()

        print(f"[STATE] {state} | Dist: {distance_traveled:.3f}m | Force: {f_soil:.1f}N | Depth: {depth*1000:.1f}mm | V_sense: {v_sense:.3f}V")

        # Immediate Obstacle Retraction Trigger
        if f_soil >= FORCE_THRESHOLD and state in ["EXTEND", "SAMPLE_HOLD"]:
            print(f'[Warning] Hard Object Detected ({f_soil:.1f}N)! Emergency Retracting...')
            state = "RETRACT"
            phase_start_time = currentTime

        # State Machine Logic
        if state == "DRIVE":
            if distance_traveled >= 0.075:
                print("[INFO] Reached sample point. Stopping robot to deploy probe.")
                state = "EXTEND"
                phase_start_time = currentTime
            else:
                for m in motors.values():
                    m.setVelocity(1.0)
                Actuator_Device.setPosition(0.0)
                Actuator_Device.setVelocity(0.01)

        elif state == "EXTEND":
            # Stop wheels completely while probing
            for m in motors.values():
                m.setVelocity(0.0)
            
            # Lower the linear actuator to target depth (-0.042m)
            Actuator_Device.setPosition(-0.042)  
            Actuator_Device.setVelocity(0.03)

            # Check if target depth (~38-40mm) has been reached
            if depth >= 0.038:
                print("[INFO] Target depth reached. Sampling site arrived. Holding to collect sample...")
                state = "SAMPLE_HOLD"
                phase_start_time = currentTime

        elif state == "SAMPLE_HOLD":
            # Keep wheels and actuator still while taking the reading
            for m in motors.values():
                m.setVelocity(0.0)
            Actuator_Device.setVelocity(0.0)

            # Hold for 3 seconds to complete sampling
            if currentTime - phase_start_time >= 3.0:
                print("[INFO] Sampling done. Retracting probe.")
                state = "RETRACT"
                phase_start_time = currentTime

        elif state == "RETRACT":
            # Keep wheels stopped
            for m in motors.values():
                m.setVelocity(0.0)
            
            # Lift probe back up to home position (0.0)
            Actuator_Device.setPosition(0.0)  
            Actuator_Device.setVelocity(0.08)

            # Once retracted for 2.5 seconds, resume driving
            if currentTime - phase_start_time >= 2.5:
                print("[INFO] Probe safely retracted. Resuming driving.")
                state = "DRIVE"
                last_sample_distance = current_absolute_distance