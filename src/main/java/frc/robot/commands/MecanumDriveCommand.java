/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain.MecanumDriveSubsystem;
import frc.robot.subsystems.drivetrain.MecanumDriveSubsystem.DriveMode;
import frc.robot.subsystems.vision.LimeLightSubsystem;

public class MecanumDriveCommand extends CommandBase {

  private MecanumDriveSubsystem DriveTrain;

  private PIDController xController = new PIDController(.001, 0, 0);
  private PIDController yController = new PIDController(0.01, 0, 0);
  private PIDController zController = new PIDController(.0003, 0, 0);

  /**
   * Creates A Mecanum Drivetrain command
   * 
   * @param DriveTrain - The Mecanum Drivetrain subsystem
   */
  public MecanumDriveCommand(MecanumDriveSubsystem DriveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.DriveTrain = DriveTrain;
    addRequirements(this.DriveTrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.DriveTrain.stop();

    xController.setTolerance(0.1);
    yController.setTolerance(0.1);
    zController.setTolerance(1);

    xController.setSetpoint(0);
    yController.setSetpoint(0);
    zController.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (RobotContainer.getController().getYButton()) {
      DriveTrain.setDriveMode(DriveMode.AUTO);
      LimeLightSubsystem.getInstance().turnLightOn();

    } else {
      DriveTrain.setDriveMode(DriveMode.MANUAL);
      LimeLightSubsystem.getInstance().turnLightOff();

    }

    if (DriveTrain.getDriveMode() == DriveMode.AUTO && LimeLightSubsystem.getInstance().hasValidTarget()) {

      double[] yCorners = LimeLightSubsystem.getInstance().getYCorners();

      double leftCorner = yCorners[1];
      double rightCorner = yCorners[0];

      double x = -xController.calculate(LimeLightSubsystem.getInstance().getXAngle());
      double y = -yController.calculate(LimeLightSubsystem.getInstance().getYAngle());
      double z = zController.calculate(LimeLightSubsystem.getInstance().getSkew());

      if (leftCorner < rightCorner) {
        // Check which way we need to rotate (left or right)
        z *= -1;
      }
      // Drive using PID calculated outputs
      DriveTrain.driveCartesian(x, y, z);

    } else {
      // Drive using joysticks
      Map<String, Double> sticks = RobotContainer.getController().getSticks();
      DriveTrain.driveCartesian(sticks.get("LSX"), sticks.get("LSY"), sticks.get("RSX"));
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
