/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.MecanumDriveSubsystem.DriveMode;
import frc.robot.subsystems.vision.LimeLightSubsystem;
import frc.robot.utils.PIDFController;

public class MecanumDriveCommand extends Command {

	PIDFController xController;
	PIDFController yController;
	PIDFController zController;

	public MecanumDriveCommand() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.DriveTrain);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.DriveTrain.stop();

		xController = new PIDFController("X_DRIVE", 0.1, 0.01, 0, 0);
		yController = new PIDFController("Y_DRIVE", 0.1, 0.1, 0, 0);
		zController = new PIDFController("Z_DRIVE", 0.1, 0.001, 0, 0);

		yController.disable();
		zController.disable();

		xController.setOutputRange(-0.3, 0.3);
		yController.setOutputRange(-0.3, 0.3);
		zController.setOutputRange(-0.3, 0.3);

		xController.setInputRange(-27, 27); // Horizontal offset
		yController.setInputRange(-20.5, 20.5); // Vertical offset
		zController.setInputRange(-90, 0); // Skew or rotation

		xController.setTolerance(0.1);
		yController.setTolerance(0.1);
		zController.setTolerance(1);

		xController.setSetPoint(0);
		yController.setSetPoint(0);
		zController.setSetPoint(0);

		xController.outputTelemetry();
		yController.outputTelemetry();
		zController.outputTelemetry();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {

		if (Robot.m_oi.getController().getYButton()) {
			Robot.DriveTrain.setDriveMode(DriveMode.AUTO);
			LimeLightSubsystem.getInstance().turnLightOn();

		} else {
			Robot.DriveTrain.setDriveMode(DriveMode.MANUAL);
			LimeLightSubsystem.getInstance().turnLightOff();

		}

		if (Robot.DriveTrain.getDriveMode() == DriveMode.AUTO && LimeLightSubsystem.getInstance().hasValidTarget()) {

			double[] yCorners = LimeLightSubsystem.getInstance().getYCorners();

			double leftCorner = yCorners[1];
			double rightCorner = yCorners[0];

			double x = !xController.onTarget() ? -xController.calculate(LimeLightSubsystem.getInstance().getXAngle())
					: 0;
			double y = !yController.onTarget() ? -yController.calculate(LimeLightSubsystem.getInstance().getYAngle())
					: 0;
			double z = !zController.onTarget() ? zController.calculate(LimeLightSubsystem.getInstance().getSkew()) : 0;

			if (leftCorner < rightCorner) {
				// Check which way we need to rotate (left or right)
				z *= -1;
			}
			// Drive using PID calculated outputs
			Robot.DriveTrain.driveCartesian(x, y, z);

		} else {
			// Drive using joysticks
			Map<String, Double> sticks = Robot.m_oi.getController().getSticks();
			Robot.DriveTrain.driveCartesian(sticks.get("LSX"), sticks.get("LSY"), sticks.get("RSX"));
		}

		xController.readTelemetry();
		yController.readTelemetry();
		zController.readTelemetry();

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.DriveTrain.stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
