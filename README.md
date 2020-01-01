# The Digital Eagles Team 5667 2019 Code Ported to 2020

### [Robot](src/main/java/frc/robot/Robot.java)

### [RobotContainer](src/main/java/frc/robot/RobotContainer.java)

### [Constants](src/main/java/frc/robot/Constants.java)

### [Subsystems](src/main/java/frc/robot/subsystems)

* [DriveTrain](src/main/java/frc/robot/subsystems/drivetrain/MecanumDriveSubsystem.java)
  * Mecanum drive subsystem
* [Elevator](src/main/java/frc/robot/subsystems/elevator/ElevatorSubsystem.java)
  * Elevator subsystem with PIDF controller implemented
* [Cargo Intake](src/main/java/frc/robot/subsystems/intakes/CargoSubsystem.java)
  * The dog cone intake subsystem
* [Hatch Intake](src/main/java/frc/robot/subsystems/intakes/HatchSubystem.java)
  * The triangle intake subsystem
* [Cameras](src/main/java/frc/robot/subsystems/vision/CameraSubsystem.java)
  * The multi-camera subsystem for driver vision
* [Lime Light Driver](src/main/java/frc/robot/subsystems/vision/LimeLightSubsystem.java)
  * Lime Light Driver written as a singleton

### [Commands](src/main/java/frc/robot/commands)

* [Drive Train Command](src/main/java/frc/robot/commands/MecanumDriveCommand.java)
  * Manual control and auto alignment using PID Control and the LimeLight
* [Elevator Command](src/main/java/frc/robot/commands/ElevatorCommand.java)
  * Manual control and PID Control
* [Cargo Command](src/main/java/frc/robot/commands/CargoCommand.java)
* [Hatch Command](src/main/java/frc/robot/commands/HatchCommand.java)

### [Utils](src/main/java/frc/robot/utils)

* [XBOX Controller](src/main/java/frc/robot/utils/Controller.java)
  * Custom XBOX controller class for added functionality
* [PIDF Controller](src/main/java/frc/robot/utils/PIDFController.java)
  * Custom PIDF Controller


Visit our site at [nahsrobotics.org](https://nahsrobotics.org)

Written, cleaned, and documented by [Rafael Piloto](https://rafaelpiloto10.herokuapp.com/)
