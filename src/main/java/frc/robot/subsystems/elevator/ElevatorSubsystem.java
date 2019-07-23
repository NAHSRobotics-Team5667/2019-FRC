/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.ElevatorCommand;
import frc.robot.subsystems.elevator.ElevatorConstants.DriveModes;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorDirection;
import frc.robot.subsystems.elevator.ElevatorConstants.Levels;

/**
 * A Linear elevator elevator subsystem
 */
public class ElevatorSubsystem extends Subsystem {
	// Constants
	private final double k_PULLEY_CIRCUMFERENCE = .102; // In meters
	private final double k_GEAR_RATIO = 64;
	private final double k_PULSES_PER_REVOLUTION = 1024;
	// Rocket levels based on height in relation to encoder ticks
	private double[] d_levels = { 0, .95, 1.45 };

	// The rocket levels

	private DriveModes driveMode = DriveModes.MANUAL;

	private PWMTalonSRX m_motor; // The elevator motor

	private double m_speedUp = 1; // The motor speed when the elevator is traveling up
	private double m_speedDown = -.7; // The motor speed when elevator is traveling down

	private Levels m_level = Levels.ONE;

	private Encoder encoder;

	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new ElevatorCommand());
	}

	/**
	 * A linear elevator elevator subsystem
	 * 
	 * @param id         - The linear elevator motor PWM port on the roborio
	 * @param isInverted - Is the motor inverted
	 * @param speedUp    - The motor speed when the elevator is traveling up
	 * @param speedDown  - The motor speed when elevator is traveling down
	 * @param d_levels   - The height in meters in relation to encoder ticks for
	 *                   each level of the rocket.
	 * @param encoder    - The elevator encoder
	 */
	public ElevatorSubsystem(int id, boolean isInverted, double speedUp, double speedDown, double[] d_levels,
			Encoder encoder) {
		this.m_motor = new PWMTalonSRX(id);
		this.m_motor.setInverted(isInverted);

		this.m_speedUp = speedUp;
		this.m_speedDown = speedDown;

		this.d_levels = d_levels;

		this.encoder = encoder;

	}

	/**
	 * A linear elevator elevator subsystem
	 * 
	 * @param id         - The linear elevator motor PWM port on the roborio
	 * @param isInverted - Is the motor inverted
	 * @param encoder    - The elevator encoder
	 */
	public ElevatorSubsystem(int id, boolean isInverted, Encoder encoder) {
		this.m_motor = new PWMTalonSRX(id);
		this.m_motor.setInverted(isInverted);
		this.encoder = encoder;

	}

	/**
	 * A linear elevator elevator subsystem
	 * 
	 * @param id         - The linear elevator motor PWM port on the roborio
	 * @param isInverted - Is the motor inverted
	 * 
	 **/
	public ElevatorSubsystem(int id, boolean isInverted) {
		this.m_motor = new PWMTalonSRX(id);
		this.m_motor.setInverted(isInverted);
	}

	/**
	 * A Linear elevator elevator subsystem
	 * 
	 * @param id - The linear elevator motor PWM port on the roborio
	 */
	public ElevatorSubsystem(int id) {
		this.m_motor = new PWMTalonSRX(id);
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
		switch (level) {
		case ONE:
			return this.d_levels[0];
		case TWO:
			return this.d_levels[1];
		case THREE:
			return this.d_levels[2];
		default:
			return d_levels[0];
		}
	}

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
	 * Dirve the elevator using direction
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
		if (targetHeight > currentHeight) {
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

	public void increaseLevel() {
		setCurrentLevel((m_level == Levels.ONE) ? Levels.TWO : Levels.THREE);
	}

	public void decreaseLevel() {
		Levels level = (m_level == Levels.THREE) ? Levels.TWO : Levels.ONE;
		if (m_level != level) {
			setCurrentLevel(level);
		}
	}

	/**
	 * Set the current auto mode
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
		return this.driveMode;
	}

	public Levels getCurrentLevel() {
		return m_level;
	}

}
