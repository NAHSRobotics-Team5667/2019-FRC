/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.utils.PIDFController;

public class ElevatorCommand extends Command {

    private PIDFController pidfController = new PIDFController("Elevator", 3.5, 0, 0, .1);

    public ElevatorCommand() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.Elevator);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        Robot.Elevator.stop();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        // Manual Elevator drive
        if (Robot.m_oi.getController().getRightTrigger() != 0) {
            Robot.Elevator.setDriveMode(ElevatorConstants.DriveModes.MANUAL);
            Robot.Elevator.driveElevatorByDirection(ElevatorConstants.ElevatorDirection.UP);

        } else if (Robot.m_oi.getController().getLeftTrigger() != 0) {
            Robot.Elevator.setDriveMode(ElevatorConstants.DriveModes.MANUAL);
            Robot.Elevator.driveElevatorByDirection(ElevatorConstants.ElevatorDirection.DOWN);

        } else { // Auto PID Elevator drive

            if (Robot.m_oi.getController().getRightBumperPressed()) {
                Robot.Elevator.setDriveMode(ElevatorConstants.DriveModes.AUTO);
                Robot.Elevator.increaseLevel();
                pidfController.setSetPoint(Robot.Elevator.getHeightFromLevel(Robot.Elevator.getCurrentLevel()));

            } else if (Robot.m_oi.getController().getLeftBumperPressed()) {
                Robot.Elevator.setDriveMode(ElevatorConstants.DriveModes.AUTO);
                Robot.Elevator.decreaseLevel();
                pidfController.setSetPoint(Robot.Elevator.getHeightFromLevel(Robot.Elevator.getCurrentLevel()));

            } else if (Robot.Elevator.getDriveMode() == ElevatorConstants.DriveModes.MANUAL) {
                Robot.Elevator.stop();

            } else {
                Robot.Elevator.driveElevatorBySpeed(pidfController.calculate(Robot.Elevator.getCurrentHeight()));
            }
        }

        if (Robot.m_oi.getController().getXButtonPressed()) {
            Robot.Elevator.setDriveMode(ElevatorConstants.DriveModes.MANUAL);
            Robot.Elevator.stop();
            Robot.Elevator.resetEncoder();
        }

    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.Elevator.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}
