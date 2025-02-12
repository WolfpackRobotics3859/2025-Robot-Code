// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main 
{
  private Main() 
  {
    //empty for now
  }

  public static void main(String... args) 
  {
    RobotBase.startRobot(() -> {
      try {
          return new Robot();
      } catch (FileVersionException | IOException | ParseException e) {
          e.printStackTrace();
          throw new RuntimeException("[CRITICAL ERROR] Failed to initialize Robot!", e);
      }
  });
  }
}
