// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;

public class CoralPlacer extends SequentialCommandGroup {

  public CoralPlacer(Elevator elevator, double elevatorLevel, Shooter shooter) {
    addCommands(
      elevator.goToPosition(elevatorLevel), //move to the requested level
      shooter.applyShooterVoltage(ShooterConstants.SHOOTER_CORAL_DEPLOY_VOLTAGE), //deploy coral
      new WaitCommand(1), //lets shooter run for 1 second before lowering the elevator
      //wait command can  be removed later when  the shooter is able to stop itself based on sensor feedback
      elevator.goToPosition(ElevatorConstants.ELEVATOR_ZERO_POSITION) //move to zero position
    );
  }
}

