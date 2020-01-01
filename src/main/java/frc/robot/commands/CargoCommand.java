/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.robot.Robot;

/**
 * The hatch mech outake command
 */
public class CargoCommand extends TimedCommand {
	/**
	 * The hatch outake command which automatically opens and closes the piston
	 */
	public CargoCommand(double timeout) {
		super(timeout);
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.CargoIntake);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.CargoIntake.enablePiston();
		Robot.CargoIntake.outputTelemetry();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		Robot.CargoIntake.outputTelemetry();
	}

	// Called once after timeout
	@Override
	protected void end() {
		Robot.CargoIntake.disablePiston();
		Robot.CargoIntake.outputTelemetry();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
