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
    ; // Java bug? Need some value
    public enum Levels {
        ONE(1), TWO(2), THREE(3);

        private int level;

        /**
         * The Elevator Level
         * 
         * @param level - Level in relation to Rocket levels
         */
        private Levels(int level) {
            this.level = level;
        }

        /**
         * Get the level as an integer
         * 
         * @return Level as an integer
         */
        public int getLevel() {
            return level;
        }
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
