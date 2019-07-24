/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.elevator;

/**
 * Elevator Constants
 */
public enum ElevatorConstants {
    ; // Java bug :/
    public enum Levels {
        ONE, TWO, THREE;
    }

    // The direction the elevator can travel in
    public enum ElevatorDirection {
        UP, DOWN;
    }

    // Robot drive modes available
    public enum DriveModes {
        MANUAL, AUTO;
    }
}
