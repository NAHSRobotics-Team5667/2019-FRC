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

	private static Controller m_controller;

	private Button m_aButton;
	private Button m_bButton;

	/**
	 * The Operator Interface
	 * 
	 * @param controller_port - Port for XBox controller (default 0)
	 */
	public OI(int controller_port) {
		m_controller = new Controller(controller_port);

		m_aButton = new JoystickButton(m_controller, RobotMap.button_A_Port);
		m_bButton = new JoystickButton(m_controller, RobotMap.button_B_Port);

		m_aButton.whenPressed(new HatchCommand(RobotMap.HatchOutakeTime));
		m_bButton.whenPressed(new CargoCommand(RobotMap.CargoOutakeTime));
	}

	/**
	 * Get the XBox Controller Instance
	 * 
	 * @return the XBox Controller Instance
	 */
	public Controller getController() {
		return m_controller;
	}
}
