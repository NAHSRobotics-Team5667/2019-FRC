/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.subsystems.intakes.HatchSubsystem;

/**
 * Hatch Intake JUnit tests
 */
public class HatchIntakeTest {

    @Test
    public void hatchIntakeTestTrue() {
        // -------- Hatch Subsystem Test 1 --------
        HatchSubsystem hatchSubsystem = new HatchSubsystem(new Solenoid(0), true);

        hatchSubsystem.enablePiston();
        assertEquals(hatchSubsystem.getStatus(), true);

        hatchSubsystem.disablePiston();
        assertEquals(hatchSubsystem.getStatus(), false);

        hatchSubsystem.close();
    }

    @Test
    public void hatchIntakeTestFalse() {
        // -------- Hatch Subsystem Test 2 --------
        HatchSubsystem hatchSubsystem = new HatchSubsystem(new Solenoid(1), false);

        hatchSubsystem.enablePiston();
        assertEquals(hatchSubsystem.getStatus(), false);

        hatchSubsystem.disablePiston();
        assertEquals(hatchSubsystem.getStatus(), true);

        hatchSubsystem.close();

    }

}
