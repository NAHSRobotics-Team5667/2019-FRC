/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.intakes;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * The Robot Hatch Intake
 */
public class HatchSubsystem extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	private Solenoid piston;
	private boolean kOn;
	private boolean kOff;

	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	/**
	 * The hatch subsystem constructor
	 * 
	 * @param piston - The solenoid which controls the hatch mech
	 * @param setOn  - The value which should be used for enabling / firing the
	 *               piston
	 */
	public HatchSubsystem(Solenoid piston, boolean setOn) {
		this.piston = piston;
		this.kOn = setOn;
		this.kOff = !setOn;

		this.disablePiston(); // Make sure the piston is always closed at the start of a match
	}

	/**
	 * The hatch subsystem constructor with a default value of true for enabling /
	 * firing the piston
	 * 
	 * @param piston - The solenoid which controls the hatch mech
	 */
	public HatchSubsystem(Solenoid piston) {
		this(piston, true);
	}

	public void enablePiston() {
		piston.set(kOn);
	}

	public void disablePiston() {
		piston.set(kOff);
	}

	public void togglePiston() {
		piston.set(!piston.get());
	}
}
