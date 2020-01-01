/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.intakes;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The Robot Cargo Intake
 */
public class CargoSubsystem extends SubsystemBase {
  private Solenoid piston;
  private boolean kOn;
  private boolean kOff;

  /**
   * The cargo subsystem constructor
   * 
   * @param piston - The solenoid which controls the cargo mech
   * @param setOn  - The value which should be used for enabling / firing the
   *               piston
   */
  public CargoSubsystem(Solenoid piston, boolean setOn) {
    this.piston = piston;
    this.kOn = setOn;
    this.kOff = !setOn;

    this.disablePiston(); // Make sure the piston is always closed at the start of a match
  }

  /**
   * The cargo subsystem constructor with a default value of true for enabling /
   * firing the piston
   * 
   * @param piston - The solenoid which controls the cargo mech
   */
  public CargoSubsystem(Solenoid piston) {
    this(piston, true);
  }

  /**
   * Enables the solenoid
   */
  public void enablePiston() {
    piston.set(kOn);
  }

  /**
   * Disables the solenoid
   */
  public void disablePiston() {
    piston.set(kOff);
  }

  /**
   * Toggles the solenoid
   */
  public void togglePiston() {
    piston.set(!piston.get());
  }

  /**
   * Get the status of the solenoid
   * 
   * @return The status of the solenoid
   */
  public boolean getStatus() {
    return this.piston.get();
  }

  /**
   * Output diagnostics
   */
  public void outputTelemetry() {
    SmartDashboard.putBoolean("Cargo:", getStatus());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
