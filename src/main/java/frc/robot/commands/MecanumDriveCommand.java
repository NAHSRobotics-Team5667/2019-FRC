/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain.MecanumDriveSubsystem;
import frc.robot.subsystems.drivetrain.MecanumDriveSubsystem.DriveMode;
import frc.robot.subsystems.vision.LimeLightSubsystem;
import frc.robot.utils.PIDFController;

public class MecanumDriveCommand extends CommandBase {

  private MecanumDriveSubsystem DriveTrain;

  private PIDFController xController;
  private PIDFController yController;
  private PIDFController zController;

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

    /**
     * Set PID Constants (Need to refactor to use new PID class)
     */
    xController = new PIDFController("X_DRIVE", 0.001, 0, 0, 0);
    yController = new PIDFController("Y_DRIVE", 0.01, 0, 0, 0);
    zController = new PIDFController("Z_DRIVE", 0.003, 0, 0, 0);

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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xController.outputTelemetry();
    yController.outputTelemetry();
    zController.outputTelemetry();

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

    xController.readTelemetry();
    yController.readTelemetry();
    zController.readTelemetry();

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
