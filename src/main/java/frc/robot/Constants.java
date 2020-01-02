/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final static class ElevatorConstants {
        // ------------------- PWM Ports ---------------------------
        public static final int ElevatorPWM = 8;
        // ------------------- DIO Ports ---------------------------
        public static int ElevatorEncoderA = 8;
        public static int ElevatorEncoderB = 9;
        // ---------------- Elevator Defaults -----------------------
        public static final double e_SPEEDUP = 1; // The motor speed when the elevator is traveling up
        public static final double e_SPEEDDOWN = -.7; // The motor speed when elevator is traveling down
        public static final double[] e_LEVELS = { .1, .95, 1.45 }; // Rocket heights in meters
        public static final double kP = 3.5, kI = 0, kD = 0, kF = .1;

    }

    public final static class DriveTrainConstants {
        // ------------------- PWM Ports ---------------------------
        public static final int frontRightMotor = 0;
        public static final int rearRightMotor = 1;
        public static final int frontLeftMotor = 2;
        public static final int rearLeftMotor = 3;
    }

    public final static class IntakeConstants {
        // ------------------- PCM Ports ---------------------------
        // Hatch Piston
        public static final int HatchSolenoid = 0;
        // Cargo Piston
        public static final int CargoSolenoid = 5;

        // ------------------- Auto Times --------------------------
        // Hatch Piston Time
        public static final int HatchOutakeTime = 3; // 3 seconds
        // Cargo Piston Time
        public static final int CargoOutakeTime = 3; // 3 seconds

    }

    public final static class VisionConstants {
        // ---------------- Camera Defaults -----------------------
        public static final int c_WIDTH = 120;
        public static final int c_HEIGHT = 60;
        public static final int c_FPS = 18;
    }

    public final static class ControllerConstants {
        public static final int controllerPort = 0; // Controller port

        // Sticks
        public static final int sRightX_Port = 4; // Right stick x
        public static final int sRightY_Port = 5; // Right stick y
        public static final int sLeftX_Port = 0; // Left stick x
        public static final int sLeftY_Port = 1; // Left stick y

        // Triggers
        public static final int TriggerRight_Port = 3; // Right trigger
        public static final int TriggerLeft_Port = 2; // Left trigger

        // Bumpers
        public static final int BumperRight_Port = 6; // Right bumper
        public static final int BumperLeft_Port = 5; // Left bumper

        // Buttons
        public static final int button_A_Port = 1; // A Button
        public static final int button_B_Port = 2; // B Button
        public static final int button_X_Port = 3; // X Button
        public static final int button_Y_Port = 4; // Y Button

        // Special buttons
        public static final int button_menu_Port = 8; // Menu Button
        public static final int button_Start_Port = 7; // Start button

    }
}
