// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.groups;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.CoralPurge;
// import frc.robot.subsystems.CoralPlacer;

// /**
//  * NOTE:  Consider using this command inline, rather than writing a subclass.  For more
//  * information, see:
//  * https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
//  */
// public class PlaceCoralOnReef extends SequentialCommandGroup 
// {
//   CoralPlacer m_CoralPlacer;
//   /** 
//    * Creates a new PlacerCoralOnReef.
//    * 
//    * @param p_CoralPlacer CoralPlacer subsystem created in robotcontainer
//    * @param p_ElevatorlLevel ElevatorLevel command created in robocontainer
//    */
//   public PlaceCoralOnReef(CoralPlacer p_CoralPlacer, Command p_ElevatorlLevel) 
//   {
//     m_CoralPlacer = p_CoralPlacer;
//     addCommands
//     (
//       new CoralPurge(m_CoralPlacer),
//       p_ElevatorlLevel
//     );
//   }
// }
