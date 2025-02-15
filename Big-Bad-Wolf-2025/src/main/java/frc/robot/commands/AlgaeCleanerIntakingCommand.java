// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.constants.ShooterConstants;
// import frc.robot.subsystems.Shooter;

// // Creates a new Algae Cleaner Intaking command.
// public class AlgaeCleanerIntakingCommand extends Command 
// {
//   private final Shooter m_Shooter;

//   /**
//    * Algae Cleaner command constructor.
//    * @param algaeCleaner Algae Cleaner subsystem.
//    */
//   public AlgaeCleanerIntakingCommand(Shooter shooter) 
//   {
//     this.m_Shooter = shooter;
//     addRequirements(m_Shooter);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() 
//   {
//     m_Shooter.setShooterWristPosition(ShooterConstants.WRIST_CORAL_DEPLOY_POSITION);
//     m_Shooter.setShooterRollerVoltage(ShooterConstants.CORAL_INTAKING_ROLLER_VOLTAGE);  
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() 
//   {
//     // Intentionally Empty.
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) 
//   {
//     // Intentionally Empty.
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() 
//   {
//     return false;
//   }
// }