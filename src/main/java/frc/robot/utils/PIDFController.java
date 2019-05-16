/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

/**
 * A custom Closed-Loop Feedback PIDF Controller
 */
public class PIDFController {

    private String name;
    private double kP, kI, kD, kF, setpoint;

    /**
     * The PID Controller constructor
     * 
     * @param name - The Subsytem name
     * @param kP   - the Proportional gain
     * @param kI   - the Integral gain
     * @param kD   - the Derivative gain
     * @param kF   - the Feed Forward gain
     */
    public PIDFController(String name, double kP, double kI, double kD, double kF) {
        this.name = name;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    /**
     * Create a blank PID Controller
     * 
     * @param name - The subsystem name
     */
    public PIDFController(String name) {
        this(name, 0, 0, 0, 0);
    }

    /**
     * A P Controller
     * 
     * @param name - the subsystem name
     * @param kP   - the proportional gain
     */
    public PIDFController(String name, double kP) {
        this(name, kP, 0, 0, 0);
    }

    /**
     * A PI Controller
     * 
     * @param name - the subsystem name
     * @param kP   - the proportional gain
     * @param kI   - the integral gain
     */
    public PIDFController(String name, double kP, double kI) {
        this(name, kP, kI, 0, 0);
    }

    /**
     * A PID Controller
     * 
     * @param name - the subsystem name
     * @param kP   - the proportional gain
     * @param kI   - the integral gain
     * @param kD   - the derivative gain
     */
    public PIDFController(String name, double kP, double kI, double kD) {
        this(name, kP, kI, kD, 0);
    }

    /**
     * Set the proportinal gain
     * 
     * @param kP - the proportional gain
     */
    public void setP(double kP) {
        this.kP = kP;
    }

    /**
     * Set the integral gain
     * 
     * @param kI - the integral gain
     */
    public void setI(double kI) {
        this.kI = kI;
    }

    /**
     * Set the derivative gain
     * 
     * @param kD - the derivative gain
     */
    public void setD(double kD) {
        this.kD = kD;
    }

    /**
     * Set the feed forward gain
     * 
     * @param kF - the feed forward gain
     */
    public void setF(double kF) {
        this.kF = kF;
    }

    /**
     * Set the setpoint (aka target)
     * 
     * @param setpoint - The setpoint
     */
    public void setSetPoint(double setpoint) {
        this.setpoint = setpoint;
    }

    /**
     * Calculate the output
     * 
     * @param input - The sensor input
     * @return The output
     */
    public double calculate(double input) {
        double error = this.setpoint - input;
        return kF + ((error * kP) + (error * kI) + (error * kD));
    }

}
