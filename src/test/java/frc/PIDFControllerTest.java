/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package test;

import static org.junit.Assert.assertEquals;

import org.junit.Before;
import org.junit.Test;

import frc.robot.utils.PIDFController;
import frc.robot.utils.PIDFController.OutputFormat;

/**
 * PIDF Controller JUnit tests
 */
public class PIDFControllerTest {

    private PIDFController pidfController = new PIDFController("test");
    private double kP = 0.1;
    private double kI = 0;
    private double kD = 0;

    private double m_setPoint = 0;

    @Before
    public void setup() {
        pidfController.setP(kP);
        pidfController.setI(kI);
        pidfController.setD(kD);

        pidfController.setInputRange(-100, 100);
        pidfController.setOutputRange(-1, 1);
        pidfController.setSetPoint(m_setPoint);
        pidfController.setTolerance(.1);
    }

    @Test
    public void kPGainTest() {
        pidfController.setOutputFormat(OutputFormat.MAP);

        double output1 = pidfController.calculate(100);
        double output2 = pidfController.calculate(-100);

        double output3 = pidfController.calculate(50);
        double output4 = pidfController.calculate(-50);

        System.out.println(String.format("Outputs: %f, %f, %f, %f", output1, output2, output3, output4));

        assertEquals(output1 == -1, true);
        assertEquals(output2 == 1, true);
        // assertEquals(output3 == -0.05, true);
        // assertEquals(output4 == 0.05, true);
    }

}
