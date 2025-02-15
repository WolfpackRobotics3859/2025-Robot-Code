// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceCoralLevelFour extends SequentialCommandGroup {
  /** Creates a new PlaceCoralLevelFour. */
  Shooter m_Shooter;
  public PlaceCoralLevelFour(Shooter p_Shooter, Command p_ElevatorLevel) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_Shooter = p_Shooter;
    addCommands(
      m_Shooter.SetLevelFourCoralPlacer(ShooterConstants.SHOOTER_SHOOTING_ROLLER_VOLTAGE, ShooterConstants.CORAL_PLACER_LEVEL_FOUR_POSITION),
      p_ElevatorLevel
    );
  }
}
