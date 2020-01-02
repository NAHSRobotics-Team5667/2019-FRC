/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.ElevatorConstants.DriveModes;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorDirection;
import frc.robot.subsystems.elevator.ElevatorConstants.Levels;

public class ElevatorPIDSubsystem extends PIDSubsystem {

  // Constants
  private final double k_PULLEY_CIRCUMFERENCE = .102; // In meters
  private final double k_GEAR_RATIO = 64;
  private final double k_PULSES_PER_REVOLUTION = 1024;
  // Rocket levels based on height in relation to encoder ticks and meters
  private double[] d_levels;

  // The rocket levels

  private DriveModes driveMode = DriveModes.MANUAL;

  private SpeedController m_motor; // The elevator motor

  private double m_speedUp = Constants.ElevatorConstants.e_SPEEDUP; // The motor speed when the elevator is traveling up
  private double m_speedDown = Constants.ElevatorConstants.e_SPEEDDOWN; // The motor speed when elevator is traveling
                                                                        // down

  private Levels m_level = Levels.ONE;

  private Encoder m_encoder;

  private double kF;

  /**
   * A linear elevator elevator subsystem
   * 
   * @param motor      - The linear elevator motor
   * @param isInverted - Is the motor inverted
   * @param d_levels   - The height in meters in relation to encoder ticks for
   *                   each level of the rocket.
   * @param encoder    - The elevator encoder
   * @param kP         - the Proportional gain
   * @param kI         - the Integral gain
   * @param kD         - the Derivative gain
   * @param kF         - the Feed Forward gain
   */
  public ElevatorPIDSubsystem(SpeedController motor, boolean isInverted, double[] d_levels, Encoder encoder, double kP,
      double kI, double kD, double kF) {
    // The PID Controller used by the class
    super(new PIDController(kP, kI, kD));

    m_motor = motor;
    m_motor.setInverted(isInverted);

    this.d_levels = d_levels;

    m_encoder = encoder;

    if (m_encoder != null) {
      resetEncoder();
      m_encoder.setReverseDirection(true);
    }

    this.kF = kF;

  }

  /**
   * reset the encoder
   */
  public void resetEncoder() {
    m_encoder.reset();
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    driveElevatorBySpeed(kF + output);
  }

  /**
   * Drive the elevator using a custom speed
   * 
   * @param speed - The elevator speed
   */
  public void driveElevatorBySpeed(double speed) {
    m_motor.set(speed);
  }

  /**
   * Set the current elevator level based on rocket levels
   * 
   * @param level - The level the elevator is currently on
   */
  public void setCurrentLevel(Levels level) {
    m_level = level;
  }

  /**
   * Increase the elevator level counter
   */
  public void increaseLevel() {
    setCurrentLevel((m_level == Levels.ONE) ? Levels.TWO : Levels.THREE);
  }

  /**
   * Decrease the elevator level counter
   */
  public void decreaseLevel() {
    setCurrentLevel((m_level == Levels.THREE) ? Levels.TWO : Levels.ONE);
  }

  /**
   * Stop the elevator motor
   */
  public void stop() {
    this.m_motor.stopMotor();
  }

  /**
   * Set the current elevator drive mode
   * 
   * @param mode - The Drive mode for the elevator (MANUAL or AUTO)
   */
  public void setDriveMode(DriveModes mode) {
    this.driveMode = mode;
  }

  /**
   * Get the elevator's current drive mode
   * 
   * @return The elevator's current drive mode (MANUAL or AUTO)
   */
  public DriveModes getDriveMode() {
    return driveMode;
  }

  /**
   * Get the elevator's current level
   * 
   * @return The Elevator's current level
   */
  public Levels getCurrentLevel() {
    return m_level;
  }

  /**
   * Get the elevator's current level in an integer format
   * 
   * @return The Elevator's current level as an integer
   */
  public int getCurrentLevelNum() {
    return m_level.getLevel();
  }

  /**
   * Drive the elevator using direction
   * 
   * @param dir - The direction the elevator will be going in
   */
  public void driveElevatorByDirection(ElevatorDirection dir) {
    this.m_motor.set((dir == ElevatorDirection.UP) ? m_speedUp : m_speedDown);
  }

  /**
   * Get the height for the level requested
   * 
   * @param level - The level requested
   * @return The height in meters in relation to the encoder ticks for the
   *         elevator
   */
  public double getHeightFromLevel(Levels level) {
    return d_levels[level.getLevel() - 1];
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return (m_encoder.get() / (k_PULSES_PER_REVOLUTION * k_GEAR_RATIO)) * k_PULLEY_CIRCUMFERENCE;
  }
}
