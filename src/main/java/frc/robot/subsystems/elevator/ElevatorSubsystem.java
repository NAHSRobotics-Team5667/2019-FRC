/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.ElevatorConstants.DriveModes;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorDirection;
import frc.robot.subsystems.elevator.ElevatorConstants.Levels;

/**
 * A Linear elevator elevator subsystem
 */
public class ElevatorSubsystem extends SubsystemBase {

  // Constants
  private final double k_PULLEY_CIRCUMFERENCE = .102; // In meters
  private final double k_GEAR_RATIO = 64;
  private final double k_PULSES_PER_REVOLUTION = 1024;
  // Rocket levels based on height in relation to encoder ticks and meters
  private double[] d_levels;

  // The rocket levels

  private DriveModes driveMode = DriveModes.MANUAL;

  private SpeedController m_motor; // The elevator motor

  private double m_speedUp; // The motor speed when the elevator is traveling up
  private double m_speedDown; // The motor speed when elevator is traveling down

  private Levels m_level = Levels.ONE;

  private Encoder encoder;

  /**
   * Creates a new ElevatorSubsystem.
   */
  public ElevatorSubsystem() {

  }

  /**
   * A linear elevator elevator subsystem
   * 
   * @param motor      - The linear elevator motor
   * @param isInverted - Is the motor inverted
   * @param speedUp    - The motor speed when the elevator is traveling up
   * @param speedDown  - The motor speed when elevator is traveling down
   * @param d_levels   - The height in meters in relation to encoder ticks for
   *                   each level of the rocket.
   * @param encoder    - The elevator encoder
   */
  public ElevatorSubsystem(SpeedController motor, boolean isInverted, double speedUp, double speedDown,
      double[] d_levels, Encoder encoder) {

    this.m_motor = motor;
    this.m_motor.setInverted(isInverted);

    this.m_speedUp = speedUp;
    this.m_speedDown = speedDown;

    this.d_levels = d_levels;

    this.encoder = encoder;

    if (this.encoder != null) {
      resetEncoder();
      this.encoder.setReverseDirection(true);
    }

  }

  /**
   * A linear elevator elevator subsystem
   * 
   * @param motor      - The linear elevator motor
   * @param isInverted - Is the motor inverted
   * @param encoder    - The elevator encoder
   */
  public ElevatorSubsystem(SpeedController motor, boolean isInverted, Encoder encoder) {
    this(motor, isInverted, Constants.ElevatorConstants.e_SPEEDUP, Constants.ElevatorConstants.e_SPEEDDOWN,
        Constants.ElevatorConstants.e_LEVELS, encoder);

  }

  /**
   * A linear elevator elevator subsystem
   * 
   * @param motor      - The linear elevator motor
   * @param isInverted - Is the motor inverted
   * 
   **/
  public ElevatorSubsystem(SpeedController motor, boolean isInverted) {
    this(motor, isInverted, Constants.ElevatorConstants.e_SPEEDUP, Constants.ElevatorConstants.e_SPEEDDOWN,
        Constants.ElevatorConstants.e_LEVELS, null);

  }

  /**
   * A Linear elevator elevator subsystem
   * 
   * @param motor - The linear elevator motor
   */
  public ElevatorSubsystem(SpeedController motor) {
    this(motor, false, Constants.ElevatorConstants.e_SPEEDUP, Constants.ElevatorConstants.e_SPEEDDOWN,
        Constants.ElevatorConstants.e_LEVELS, null);

  }

  /**
   * Set the levels for the rocket based on height
   * 
   * @param d_levels - A double array where the first index is the home state and
   *                 the last index is the third level of the rocket
   */
  public void setLevels(double[] levels) {
    d_levels = levels;
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

  /**
   * Get the Elevator's height in meters using the Encoder count
   * 
   * @return the current height in meters
   */
  public double getCurrentHeight() {
    // currentEncoderPulses / (pulsesPerRevolution * gear ratio) *
    // circumferenceOfPulley
    return (this.encoder.get() / (k_PULSES_PER_REVOLUTION * k_GEAR_RATIO)) * k_PULLEY_CIRCUMFERENCE;

  }

  /**
   * Drive the elevator using a custom speed
   * 
   * @param speed - The elevator speed
   */
  public void driveElevatorBySpeed(double speed) {
    this.m_motor.set(speed);
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
   * Move the elevator based on a level you are trying to reach
   * 
   * @param level         - The desired level
   * @param currentHeight - The current height
   */
  public void driveElevatorByLevel(Levels level, double currentHeight) {
    double targetHeight = this.getHeightFromLevel(level);
    if (targetHeight < currentHeight) {
      this.driveElevatorByDirection(ElevatorDirection.DOWN);
    } else {
      this.driveElevatorByDirection(ElevatorDirection.UP);
    }
  }

  /**
   * Stop the elevator motor
   */
  public void stop() {
    this.m_motor.stopMotor();
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
   * reset the encoder
   */
  public void resetEncoder() {
    this.encoder.reset();
  }

  /**
   * Output diagnostics
   */
  public void outputTelemetry() {
    SmartDashboard.putNumber("Elevator Height:", getCurrentHeight());
    SmartDashboard.putNumber("Elevator Level:", getCurrentLevelNum());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
