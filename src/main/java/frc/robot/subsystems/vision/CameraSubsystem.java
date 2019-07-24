/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.vision;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * A modular Multi-camera subsystem
 */
public class CameraSubsystem extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	UsbCamera[] m_cameras;

	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	/**
	 * A modular multicamera subsystem
	 * 
	 * @param cameraCount - The number of cameras
	 * @param width       - The X resolution for the cameras
	 * @param height      - The Y resolution for the cameras
	 * @param fps         - The Frames Per Second for the cameras
	 */
	public CameraSubsystem(int cameraCount, int width, int height, int fps) {

		m_cameras = new UsbCamera[cameraCount];
		for (int i = 0; i < m_cameras.length; i++) {
			m_cameras[i] = CameraServer.getInstance().startAutomaticCapture(i);
			m_cameras[i].setResolution(width, height);
			m_cameras[i].setFPS(fps);
		}
	}

	/**
	 * A modular multicamera subsystem
	 * 
	 * @param cameraCount - The number of cameras
	 */
	public CameraSubsystem(int cameraCount) {
		this(cameraCount, RobotMap.c_WIDTH, RobotMap.c_HEIGHT, RobotMap.c_FPS);
	}

}
