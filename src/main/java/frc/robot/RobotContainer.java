/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CargoCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.HatchCommand;
import frc.robot.commands.MecanumDriveCommand;
import frc.robot.subsystems.drivetrain.MecanumDriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorPIDSubsystem;
import frc.robot.subsystems.intakes.CargoSubsystem;
import frc.robot.subsystems.intakes.HatchSubsystem;
import frc.robot.subsystems.vision.CameraSubsystem;
import frc.robot.subsystems.vision.LimeLightSubsystem;
import frc.robot.utils.Controller;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private static Controller m_controller = new Controller(Constants.ControllerConstants.controllerPort);

  private static MecanumDriveSubsystem Drivetrain;
  private static ElevatorPIDSubsystem Elevator;

  private static HatchSubsystem HatchIntake = new HatchSubsystem(new Solenoid(Constants.IntakeConstants.HatchSolenoid));
  private static CargoSubsystem CargoIntake = new CargoSubsystem(new Solenoid(Constants.IntakeConstants.CargoSolenoid));
  private static CameraSubsystem Cameras = new CameraSubsystem(2);

  private Button m_aButton;
  private Button m_bButton;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    Drivetrain = new MecanumDriveSubsystem(new PWMTalonSRX(Constants.DriveTrainConstants.frontLeftMotor),
        new PWMTalonSRX(Constants.DriveTrainConstants.rearLeftMotor),
        new PWMTalonSRX(Constants.DriveTrainConstants.frontRightMotor),
        new PWMTalonSRX(Constants.DriveTrainConstants.rearRightMotor));

    Elevator = new ElevatorPIDSubsystem(new PWMTalonSRX(Constants.ElevatorConstants.ElevatorPWM), false,
        Constants.ElevatorConstants.e_LEVELS,
        new Encoder(Constants.ElevatorConstants.ElevatorEncoderA, Constants.ElevatorConstants.ElevatorEncoderB),
        Constants.ElevatorConstants.kP, Constants.ElevatorConstants.kI, Constants.ElevatorConstants.kD,
        Constants.ElevatorConstants.kF);

    Drivetrain.setDefaultCommand(new MecanumDriveCommand(Drivetrain));
    Elevator.setDefaultCommand(new ElevatorCommand(Elevator));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_aButton = new JoystickButton(getController(), Constants.ControllerConstants.button_A_Port);
    m_bButton = new JoystickButton(getController(), Constants.ControllerConstants.button_B_Port);

    m_aButton.whenPressed(new HatchCommand(HatchIntake).withTimeout(Constants.IntakeConstants.HatchOutakeTime));
    m_bButton.whenPressed(new CargoCommand(CargoIntake).withTimeout(Constants.IntakeConstants.CargoOutakeTime));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }

  /**
   * Get the XBox Controller Instance
   * 
   * @return the XBox Controller Instance
   */
  public static Controller getController() {
    return m_controller;
  }

  public void outputTelemetry() {
    // Output the diagnostics for all necessary Subsystems!
    HatchIntake.outputTelemetry();
    CargoIntake.outputTelemetry();
    LimeLightSubsystem.getInstance().outputTelemetry();

  }

}
