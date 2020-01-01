/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.commands.MecanumDriveCommand;

/**
 * A Mecanum Drive Subsystem for the robot
 */
public class MecanumDriveSubsystem extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	private MecanumDrive drive;
	private DriveMode driveMode = DriveMode.DEFAULT;

	public enum DriveMode {
		DEFAULT(0), MANUAL(0), AUTO(1);

		private int mode;

		private DriveMode(int mode) {
			this.mode = mode;
		}

		/**
		 * Get the Drive Train's current drive mode
		 * 
		 * @return The current drive mode (0 - MANUAL, 1 - AUTO)
		 */
		public int getDriveMode() {
			return mode;
		}
	}

	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new MecanumDriveCommand());
	}

	/**
	 * A Mecanum Drive train subsystem for the robot
	 * 
	 * @param frontLeftMotor  - The front Left motor
	 * @param rearLeftMotor   - The back left motor
	 * @param frontRightMotor - The front right motor
	 * @param rearRightMotor  - The back right motor
	 */
	public MecanumDriveSubsystem(PWMTalonSRX frontLeftMotor, PWMTalonSRX rearLeftMotor, PWMTalonSRX frontRightMotor,
			PWMTalonSRX rearRightMotor) {
		this.drive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
	}

	/**
	 * Drive the robot using cartesian coordinates from a joystick
	 * 
	 * @param xSpeed    - The x axis (Left/Right joystick)
	 * @param ySpeed    - The y axis (Forward/Backward joystick)
	 * @param zRotation - The z axis (Rotation)
	 */
	public void driveCartesian(double xSpeed, double ySpeed, double zRotation) {
		this.drive.driveCartesian(xSpeed, ySpeed, zRotation);
	}

	/**
	 * Get the current Drive Mode
	 * 
	 * @return The current drive mode (0 - MANUAL, 1 - AUTO)
	 */
	public int getDriveModeInt() {
		return driveMode.getDriveMode();
	}

	/**
	 * Get the current Drive Mode
	 * 
	 * @return The current drive mode (0 - MANUAL, 1 - AUTO)
	 */
	public DriveMode getDriveMode() {
		return driveMode;
	}

	/**
	 * Set the Drive mode
	 * 
	 * @param mode - The desired drive mode behavior
	 */
	public void setDriveMode(DriveMode mode) {
		driveMode = mode;
	}

	/**
	 * Stop the robot from driving
	 */
	public void stop() {
		this.drive.stopMotor();
	}
}
