/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A custom Closed-Loop Feedback PIDF Controller
 */
public class PIDFController {

    private final double k_deltaTime = 0.02; // Default time in between calls

    private String name;
    private double kP, kI, kD, kF;

    private double m_setpoint;
    private double m_lastError = 0;
    private double m_totalError;
    private double m_error;
    private double m_tolerance;

    private double m_MinOutput;
    private double m_MaxOutput;

    private double m_MinInput;
    private double m_MaxInput;

    private boolean m_isEnabled;
    private SendableChooser<Boolean> m_EnabledChooser = new SendableChooser<>();

    private double m_output;

    private OutputFormat m_outputFormat;

    public enum OutputFormat {
        DEFAULT(0), CLAMP(1), MAP(2);

        private int m_format;

        /**
         * The PID Controller Output format
         * 
         * @param format - The PID Controller Output format
         */
        private OutputFormat(int format) {
            m_format = format;
        }

        /**
         * Get the PID Controller Output format
         * 
         * @return the PID Controller output format
         */
        public int getFormat() {
            return m_format;
        }
    }

    /**
     * The PID Controller constructor with specified output formatting
     * 
     * @param name         - the Subsystem name
     * @param kP           - the Proportional gain
     * @param kI           - the Integral gain
     * @param kD           - the Derivative gain
     * @param kF           - the Feed Forward gain
     * @param outputFormat - the PID Controller's Output Format
     */
    public PIDFController(String name, double kP, double kI, double kD, double kF, OutputFormat outputFormat) {
        this.name = name;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.m_isEnabled = true;
        this.m_outputFormat = outputFormat;

        m_EnabledChooser.setDefaultOption("Default", m_isEnabled);
        m_EnabledChooser.addOption("Disabled", false);
        m_EnabledChooser.addOption("Enabled", true);

    }

    /**
     * The PID Controller constructor
     * 
     * @param name - the Subsystem name
     * @param kP   - the Proportional gain
     * @param kI   - the Integral gain
     * @param kD   - the Derivative gain
     * @param kF   - the Feed Forward gain
     */
    public PIDFController(String name, double kP, double kI, double kD, double kF) {
        this(name, kP, kI, kD, kF, OutputFormat.DEFAULT);
    }

    /**
     * Create a blank PID Controller
     * 
     * @param name - The subsystem name
     */
    public PIDFController(String name) {
        this(name, 0, 0, 0, 0, OutputFormat.DEFAULT);
    }

    /**
     * A P Controller
     * 
     * @param name - the subsystem name
     * @param kP   - the proportional gain
     */
    public PIDFController(String name, double kP) {
        this(name, kP, 0, 0, 0, OutputFormat.DEFAULT);
    }

    /**
     * A PI Controller
     * 
     * @param name - the subsystem name
     * @param kP   - the proportional gain
     * @param kI   - the integral gain
     */
    public PIDFController(String name, double kP, double kI) {
        this(name, kP, kI, 0, 0, OutputFormat.DEFAULT);
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
        this(name, kP, kI, kD, 0, OutputFormat.DEFAULT);
    }

    /**
     * Set the proportional gain
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
        this.m_setpoint = setpoint;
    }

    /**
     * Clamp the value between a minimum and maximum value
     * 
     * @param n   - The number being measured
     * @param min - The minimum result
     * @param max - The maximum result
     * 
     * @return - The clamped value
     */
    private static double clamp(double n, double min, double max) {
        return Math.max(min, Math.min(n, max));
    }

    /**
     * Map a value from one range to another
     * 
     * @param n     - The value
     * @param i_min - The min of range 1
     * @param i_max - The max of range 1
     * @param o_min - The min of range 2 (output range)
     * @param o_max - The max of range 2 (output range)
     * @return translated value from range 1 to range 2
     */
    public static double map(double n, double i_min, double i_max, double o_min, double o_max) {
        return o_min + (o_max - o_min) * ((n - i_min) / (i_max - i_min));
    }

    /**
     * Map a value from input range to output range
     * 
     * @param n - The value
     * @return translated value from input range to output range
     */
    public double map(double n) {
        return map(n, m_MinInput, m_MaxInput, m_MinOutput, m_MaxOutput);
    }

    /**
     * How small the error must be considered "on target"
     * 
     * @param tolerance - The smallest acceptable error
     */
    public void setTolerance(double tolerance) {
        this.m_tolerance = tolerance;
    }

    /**
     * Get the PIDF Controller tolerance level
     * 
     * @return The PIDF Controller's Tolerance level
     */
    public double getTolerance() {
        return m_tolerance;
    }

    /**
     * Set the min and max values the PID Controller can set to
     * 
     * @param min - The smallest input value
     * @param max - The highest input value
     */
    public void setInputRange(double min, double max) {
        m_MinInput = min;
        m_MaxInput = max;

        // Update the setpoint to be in these values if changed mid run
        setSetPoint(m_setpoint);
    }

    /**
     * Set the min and max values the PID Controller can output
     * 
     * @param min - The minimum output
     * @param max - The maximum output
     */
    public void setOutputRange(double min, double max) {
        m_MinOutput = min;
        m_MaxOutput = max;
    }

    /**
     * Returns the last error recorded when calculate was last called
     * 
     * @return the last error recorded when calculate was last called
     */
    public double getError() {
        return m_error;
    }

    /**
     * Calculate the output
     * 
     * @param input - The feed back input
     * 
     * @return The output
     */
    public double calculate(double input) {
        if (!m_isEnabled) // If the PID Controller is disabled then don't calculate and return 0
            return 0;

        m_error = this.m_setpoint - input;
        double deltaError = m_error - m_lastError;
        m_totalError += m_error;

        m_output = kF + ((m_error * kP) + (m_totalError * kI) + ((deltaError / k_deltaTime) * kD));

        if (m_outputFormat == OutputFormat.CLAMP) {
            m_output = clamp(m_output, m_MinOutput, m_MaxOutput);
        } else if (m_outputFormat == OutputFormat.MAP) {
            m_output = map(m_output);
        }

        m_lastError = m_error;

        return m_output;
    }

    /**
     * Returns the if the controller is on target
     * 
     * @return is the controller on target
     */
    public boolean onTarget() {
        return Math.abs(m_error) < m_tolerance;
    }

    /**
     * Enables the PID Controller
     */
    public void enable() {
        m_isEnabled = true;
    }

    /**
     * Disables the PID Controller and all output is 0
     */
    public void disable() {
        m_isEnabled = false;
    }

    /**
     * Set the PID Controller's Output format
     * 
     * @param format - The way you would like the output to be formatted (default,
     *               clamped, mapped)
     */
    public void setOutputFormat(OutputFormat format) {
        this.m_outputFormat = format;
    }

    /**
     * Get the PID Controller's Output Format
     * 
     * @return The PID Controller's output format
     */
    public OutputFormat getOutputFormat() {
        return m_outputFormat;
    }

    /**
     * Output diagnostics
     */
    public void outputTelemetry() {
        m_EnabledChooser.setDefaultOption("Default", m_isEnabled);
        SmartDashboard.putData(name + "_PID", m_EnabledChooser);

        SmartDashboard.putNumber(name + "_P", kP);
        SmartDashboard.putNumber(name + "_I", kI);
        SmartDashboard.putNumber(name + "_D", kD);
        SmartDashboard.putNumber(name + "_F", kF);
        SmartDashboard.putNumber(name + "_O", m_output);

    }

    /**
     * Read the values inputted from the SmartDashboard
     */
    public void readTelemetry() {
        setP(SmartDashboard.getNumber(name + "_P", kP));
        setI(SmartDashboard.getNumber(name + "_I", kI));
        setD(SmartDashboard.getNumber(name + "_D", kD));
        setF(SmartDashboard.getNumber(name + "_F", kF));

        if (m_EnabledChooser.getSelected()) {
            enable();
        } else {
            disable();
        }

    }

}
