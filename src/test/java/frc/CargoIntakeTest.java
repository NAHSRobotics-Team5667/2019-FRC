/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package tests;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.subsystems.intakes.CargoSubystem;

/**
 * Cargo Intake JUnit tests
 */
public class CargoIntakeTest {

    @Test
    public void CargoIntakeTestTrue() {
        CargoSubystem cargoIntake = new CargoSubystem(new Solenoid(2), true);

        cargoIntake.enablePiston();
        assertEquals(cargoIntake.getStatus(), true);

        cargoIntake.disablePiston();
        assertEquals(cargoIntake.getStatus(), false);

        cargoIntake.close();

    }

    @Test
    public void CargoIntakeTestFalse() {
        CargoSubystem cargoIntake = new CargoSubystem(new Solenoid(3), false);

        cargoIntake.enablePiston();
        assertEquals(cargoIntake.getStatus(), false);

        cargoIntake.disablePiston();
        assertEquals(cargoIntake.getStatus(), true);

        cargoIntake.close();

    }
}
