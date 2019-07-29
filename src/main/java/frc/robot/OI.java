/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.CargoCommand;
import frc.robot.commands.HatchCommand;
import frc.robot.utils.Controller;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

	private static Controller controller;

	public OI(int controller_port) {
		controller = new Controller(controller_port);

		Button aButton = new JoystickButton(controller, RobotMap.button_A_Port);
		Button bButton = new JoystickButton(controller, RobotMap.button_B_Port);

		aButton.whenPressed(new HatchCommand(RobotMap.HatchOutakeTime));
		bButton.whenPressed(new CargoCommand(RobotMap.CargoOutakeTime));
	}

	public Controller getController() {
		return controller;
	}
}
