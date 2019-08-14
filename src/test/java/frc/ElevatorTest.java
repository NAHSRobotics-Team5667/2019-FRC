/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import edu.wpi.first.wpilibj.PWMTalonSRX;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

/**
 * Elevator JUnit tests
 */
public class ElevatorTest {

    @Test
    public void elevatorLevelIncreaseTest() {
        ElevatorSubsystem elevator = new ElevatorSubsystem(new PWMTalonSRX(0));

        elevator.increaseLevel();
        assertEquals(elevator.getCurrentLevelNum() == 2, true);

        elevator.increaseLevel();
        assertEquals(elevator.getCurrentLevelNum() == 3, true);

        elevator.increaseLevel();
        assertEquals(elevator.getCurrentLevelNum() == 3, true);

        elevator.close();
    }

    @Test
    public void elevatorLevelDecreaseTest() {
        ElevatorSubsystem elevator = new ElevatorSubsystem(new PWMTalonSRX(1));

        elevator.increaseLevel();
        elevator.increaseLevel();
        elevator.increaseLevel();

        assertEquals(elevator.getCurrentLevelNum() == 3, true);

        elevator.decreaseLevel();
        assertEquals(elevator.getCurrentLevelNum() == 2, true);

        elevator.decreaseLevel();
        assertEquals(elevator.getCurrentLevelNum() == 1, true);

        elevator.decreaseLevel();
        assertEquals(elevator.getCurrentLevelNum() == 1, true);

        elevator.close();
    }

}
