package frc.robot.utilities;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.WheelConstants.VoltageSpeeds;
import frc.robot.constants.WristConstants.ANGLES;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wheels;
import frc.robot.subsystems.Wrist;

public class CoralDeploymentBuilder 
{
    private Supplier<Command> flashingRed;
    private Supplier<Command> flashingGreen;
    private Drivetrain drivetrain;
    private Wrist wrist;
    private Wheels wheels;
    private Elevator elevator;

    private Command[] BLUE_CORAL_COMMANDS;
    private Command[] RED_CORAL_COMMANDS;

    private Command[] BLUE_CORAL_MOD_COMMANDS;
    private Command[] RED_CORAL_MOD_COMMANDS;

    private Command[] BLUE_ALGAE_COMMANDS;
    private Command[] RED_ALGAE_COMMANDS;

    public CoralDeploymentBuilder(Supplier<Command> flashingRed, Supplier<Command> flashingGreen, Drivetrain drivetrain, Wrist wrist, Wheels wheels, Elevator elevator)
    {
        this.flashingRed = flashingRed;
        this.flashingGreen = flashingGreen;
        this.drivetrain = drivetrain;
        this.wrist = wrist;
        this.wheels = wheels;
        this.elevator = elevator;

        this.BuildCommands();
        this.BuildCommandsModified();
        this.BuildAlgaeCommands();
    }

    public Command ScheduleCoralCommand()
    {
        return new InstantCommand(() -> {
            if(drivetrain.alliance == Alliance.Blue)
            {
                this.BLUE_CORAL_COMMANDS[DataStuff.GetFace().getValue()].schedule();
            }
            else
            {
                this.RED_CORAL_COMMANDS[DataStuff.GetFace().getValue()].schedule();
            }
        });
    }

    public Command ScheduleModifiedCoralCommand()
    {
        return new InstantCommand(() -> {
            if(drivetrain.alliance == Alliance.Blue)
            {
                this.BLUE_CORAL_MOD_COMMANDS[DataStuff.GetFace().getValue()].schedule();
            }
            else
            {
                this.RED_CORAL_MOD_COMMANDS[DataStuff.GetFace().getValue()].schedule();
            }
        });
    }

    public Command ScheduleAlgaeCommand()
    {
        return new InstantCommand(() -> {
            if(drivetrain.alliance == Alliance.Blue)
            {
                this.BLUE_ALGAE_COMMANDS[DataStuff.GetFace().getValue()].schedule();
            }
            else
            {
                this.RED_CORAL_COMMANDS[DataStuff.GetFace().getValue()].schedule();
            }
        });
    }

    private void BuildCommands()
    {
        this.BLUE_CORAL_COMMANDS = new Command[]
        {
            new SequentialCommandGroup(flashingRed.get(), drivetrain.PathfindToPose(DrivetrainConstants.BLUE_STAGING_POSES[0], 3, 3, 0), new ParallelCommandGroup(drivetrain.AlignCoral(), wrist.MoveToSelectedCoralDeployment(), elevator.MoveToDataLevel()), wheels.DeployCoral(), flashingGreen.get()), // 1
            new SequentialCommandGroup(flashingRed.get(), drivetrain.PathfindToPose(DrivetrainConstants.BLUE_STAGING_POSES[1], 3, 3, 0), new ParallelCommandGroup(drivetrain.AlignCoral(), wrist.MoveToSelectedCoralDeployment(), elevator.MoveToDataLevel()), wheels.DeployCoral(), flashingGreen.get()), // 2
            new SequentialCommandGroup(flashingRed.get(), drivetrain.PathfindToPose(DrivetrainConstants.BLUE_STAGING_POSES[2], 3, 3, 0), new ParallelCommandGroup(drivetrain.AlignCoral(), wrist.MoveToSelectedCoralDeployment(), elevator.MoveToDataLevel()), wheels.DeployCoral(), flashingGreen.get()), // 3
            new SequentialCommandGroup(flashingRed.get(), drivetrain.PathfindToPose(DrivetrainConstants.BLUE_STAGING_POSES[3], 3, 3, 0), new ParallelCommandGroup(drivetrain.AlignCoral(), wrist.MoveToSelectedCoralDeployment(), elevator.MoveToDataLevel()), wheels.DeployCoral(), flashingGreen.get()), // 4
            new SequentialCommandGroup(flashingRed.get(), drivetrain.PathfindToPose(DrivetrainConstants.BLUE_STAGING_POSES[4], 3, 3, 0), new ParallelCommandGroup(drivetrain.AlignCoral(), wrist.MoveToSelectedCoralDeployment(), elevator.MoveToDataLevel()), wheels.DeployCoral(), flashingGreen.get()), // 5
            new SequentialCommandGroup(flashingRed.get(), drivetrain.PathfindToPose(DrivetrainConstants.BLUE_STAGING_POSES[5], 3, 3, 0), new ParallelCommandGroup(drivetrain.AlignCoral(), wrist.MoveToSelectedCoralDeployment(), elevator.MoveToDataLevel()), wheels.DeployCoral(), flashingGreen.get())  // 6 
        };

        this.RED_CORAL_COMMANDS = new Command[]
        {
            new SequentialCommandGroup(flashingRed.get(), drivetrain.PathfindToPose(DrivetrainConstants.RED_STAGING_POSES[0], 3, 3, 0), new ParallelCommandGroup(drivetrain.AlignCoral(), wrist.MoveToSelectedCoralDeployment(), elevator.MoveToDataLevel()), wheels.DeployCoral(), flashingGreen.get()), // 1
            new SequentialCommandGroup(flashingRed.get(), drivetrain.PathfindToPose(DrivetrainConstants.RED_STAGING_POSES[1], 3, 3, 0), new ParallelCommandGroup(drivetrain.AlignCoral(), wrist.MoveToSelectedCoralDeployment(), elevator.MoveToDataLevel()), wheels.DeployCoral(), flashingGreen.get()), // 2
            new SequentialCommandGroup(flashingRed.get(), drivetrain.PathfindToPose(DrivetrainConstants.RED_STAGING_POSES[2], 3, 3, 0), new ParallelCommandGroup(drivetrain.AlignCoral(), wrist.MoveToSelectedCoralDeployment(), elevator.MoveToDataLevel()), wheels.DeployCoral(), flashingGreen.get()), // 3
            new SequentialCommandGroup(flashingRed.get(), drivetrain.PathfindToPose(DrivetrainConstants.RED_STAGING_POSES[3], 3, 3, 0), new ParallelCommandGroup(drivetrain.AlignCoral(), wrist.MoveToSelectedCoralDeployment(), elevator.MoveToDataLevel()), wheels.DeployCoral(), flashingGreen.get()), // 4
            new SequentialCommandGroup(flashingRed.get(), drivetrain.PathfindToPose(DrivetrainConstants.RED_STAGING_POSES[4], 3, 3, 0), new ParallelCommandGroup(drivetrain.AlignCoral(), wrist.MoveToSelectedCoralDeployment(), elevator.MoveToDataLevel()), wheels.DeployCoral(), flashingGreen.get()), // 5
            new SequentialCommandGroup(flashingRed.get(), drivetrain.PathfindToPose(DrivetrainConstants.RED_STAGING_POSES[5], 3, 3, 0), new ParallelCommandGroup(drivetrain.AlignCoral(), wrist.MoveToSelectedCoralDeployment(), elevator.MoveToDataLevel()), wheels.DeployCoral(), flashingGreen.get())  // 6 
        };
    }

    private void BuildAlgaeCommands()
    {
        this.BLUE_ALGAE_COMMANDS = new Command[]
        {
            new SequentialCommandGroup(flashingRed.get(), wheels.ApplyVoltage(VoltageSpeeds.SWEEP), new ParallelCommandGroup(Commands.waitSeconds(0.1), drivetrain.PathfindToPose(DrivetrainConstants.BLUE_STAGING_POSES[0], 3, 3, 0)), new ParallelDeadlineGroup(wheels.IntakeAlgae(), drivetrain.AlignCenter(), elevator.MoveToDataClean(), wrist.MoveToAngle(ANGLES.SWEEP)), flashingGreen.get()),
            new SequentialCommandGroup(flashingRed.get(), wheels.ApplyVoltage(VoltageSpeeds.SWEEP), new ParallelCommandGroup(Commands.waitSeconds(0.1), drivetrain.PathfindToPose(DrivetrainConstants.BLUE_STAGING_POSES[1], 3, 3, 0)), new ParallelDeadlineGroup(wheels.IntakeAlgae(), drivetrain.AlignCenter(), elevator.MoveToDataClean(), wrist.MoveToAngle(ANGLES.SWEEP)), flashingGreen.get()),
            new SequentialCommandGroup(flashingRed.get(), wheels.ApplyVoltage(VoltageSpeeds.SWEEP), new ParallelCommandGroup(Commands.waitSeconds(0.1), drivetrain.PathfindToPose(DrivetrainConstants.BLUE_STAGING_POSES[2], 3, 3, 0)), new ParallelDeadlineGroup(wheels.IntakeAlgae(), drivetrain.AlignCenter(), elevator.MoveToDataClean(), wrist.MoveToAngle(ANGLES.SWEEP)), flashingGreen.get()),
            new SequentialCommandGroup(flashingRed.get(), wheels.ApplyVoltage(VoltageSpeeds.SWEEP), new ParallelCommandGroup(Commands.waitSeconds(0.1), drivetrain.PathfindToPose(DrivetrainConstants.BLUE_STAGING_POSES[3], 3, 3, 0)), new ParallelDeadlineGroup(wheels.IntakeAlgae(), drivetrain.AlignCenter(), elevator.MoveToDataClean(), wrist.MoveToAngle(ANGLES.SWEEP)), flashingGreen.get()),
            new SequentialCommandGroup(flashingRed.get(), wheels.ApplyVoltage(VoltageSpeeds.SWEEP), new ParallelCommandGroup(Commands.waitSeconds(0.1), drivetrain.PathfindToPose(DrivetrainConstants.BLUE_STAGING_POSES[4], 3, 3, 0)), new ParallelDeadlineGroup(wheels.IntakeAlgae(), drivetrain.AlignCenter(), elevator.MoveToDataClean(), wrist.MoveToAngle(ANGLES.SWEEP)), flashingGreen.get()),
            new SequentialCommandGroup(flashingRed.get(), wheels.ApplyVoltage(VoltageSpeeds.SWEEP), new ParallelCommandGroup(Commands.waitSeconds(0.1), drivetrain.PathfindToPose(DrivetrainConstants.BLUE_STAGING_POSES[5], 3, 3, 0)), new ParallelDeadlineGroup(wheels.IntakeAlgae(), drivetrain.AlignCenter(), elevator.MoveToDataClean(), wrist.MoveToAngle(ANGLES.SWEEP)), flashingGreen.get()),
        };

        this.RED_ALGAE_COMMANDS = new Command[]
        {
            new SequentialCommandGroup(flashingRed.get(), wheels.ApplyVoltage(VoltageSpeeds.SWEEP), new ParallelCommandGroup(Commands.waitSeconds(0.1), drivetrain.PathfindToPose(DrivetrainConstants.RED_STAGING_POSES[0], 3, 3, 0)), new ParallelDeadlineGroup(wheels.IntakeAlgae(), drivetrain.AlignCenter(), elevator.MoveToDataClean(), wrist.MoveToAngle(ANGLES.SWEEP)), flashingGreen.get()),
            new SequentialCommandGroup(flashingRed.get(), wheels.ApplyVoltage(VoltageSpeeds.SWEEP), new ParallelCommandGroup(Commands.waitSeconds(0.1), drivetrain.PathfindToPose(DrivetrainConstants.RED_STAGING_POSES[1], 3, 3, 0)), new ParallelDeadlineGroup(wheels.IntakeAlgae(), drivetrain.AlignCenter(), elevator.MoveToDataClean(), wrist.MoveToAngle(ANGLES.SWEEP)), flashingGreen.get()),
            new SequentialCommandGroup(flashingRed.get(), wheels.ApplyVoltage(VoltageSpeeds.SWEEP), new ParallelCommandGroup(Commands.waitSeconds(0.1), drivetrain.PathfindToPose(DrivetrainConstants.RED_STAGING_POSES[2], 3, 3, 0)), new ParallelDeadlineGroup(wheels.IntakeAlgae(), drivetrain.AlignCenter(), elevator.MoveToDataClean(), wrist.MoveToAngle(ANGLES.SWEEP)), flashingGreen.get()),
            new SequentialCommandGroup(flashingRed.get(), wheels.ApplyVoltage(VoltageSpeeds.SWEEP), new ParallelCommandGroup(Commands.waitSeconds(0.1), drivetrain.PathfindToPose(DrivetrainConstants.RED_STAGING_POSES[3], 3, 3, 0)), new ParallelDeadlineGroup(wheels.IntakeAlgae(), drivetrain.AlignCenter(), elevator.MoveToDataClean(), wrist.MoveToAngle(ANGLES.SWEEP)), flashingGreen.get()),
            new SequentialCommandGroup(flashingRed.get(), wheels.ApplyVoltage(VoltageSpeeds.SWEEP), new ParallelCommandGroup(Commands.waitSeconds(0.1), drivetrain.PathfindToPose(DrivetrainConstants.RED_STAGING_POSES[4], 3, 3, 0)), new ParallelDeadlineGroup(wheels.IntakeAlgae(), drivetrain.AlignCenter(), elevator.MoveToDataClean(), wrist.MoveToAngle(ANGLES.SWEEP)), flashingGreen.get()),
            new SequentialCommandGroup(flashingRed.get(), wheels.ApplyVoltage(VoltageSpeeds.SWEEP), new ParallelCommandGroup(Commands.waitSeconds(0.1), drivetrain.PathfindToPose(DrivetrainConstants.RED_STAGING_POSES[5], 3, 3, 0)), new ParallelDeadlineGroup(wheels.IntakeAlgae(), drivetrain.AlignCenter(), elevator.MoveToDataClean(), wrist.MoveToAngle(ANGLES.SWEEP)), flashingGreen.get()),
        };
    }
    
    //new SequentialCommandGroup(flashingRed.get(), wheels.ApplyVoltage(VoltageSpeeds.SWEEP), Commands.waitSeconds(0.1), new ParallelDeadlineGroup(wheels.IntakeAlgae(), drivetrain.AlignCenter(), elevator.MoveToDataClean(), wrist.MoveToAngle(ANGLES.SWEEP)), flashingGreen.get())
    private void BuildCommandsModified()
    {
        this.BLUE_CORAL_MOD_COMMANDS = new Command[]
        {
            new SequentialCommandGroup(flashingRed.get(), drivetrain.PathfindToPose(DrivetrainConstants.BLUE_STAGING_POSES[0], 3, 3, 0), new ParallelCommandGroup(drivetrain.AlignCoralProfiled(), wrist.MoveToSelectedCoralDeployment(), elevator.MoveToDataLevel()), wheels.DeployCoral(), flashingGreen.get()), // 1
            new SequentialCommandGroup(flashingRed.get(), drivetrain.PathfindToPose(DrivetrainConstants.BLUE_STAGING_POSES[1], 3, 3, 0), new ParallelCommandGroup(drivetrain.AlignCoralProfiled(), wrist.MoveToSelectedCoralDeployment(), elevator.MoveToDataLevel()), wheels.DeployCoral(), flashingGreen.get()), // 2
            new SequentialCommandGroup(flashingRed.get(), drivetrain.PathfindToPose(DrivetrainConstants.BLUE_STAGING_POSES[2], 3, 3, 0), new ParallelCommandGroup(drivetrain.AlignCoralProfiled(), wrist.MoveToSelectedCoralDeployment(), elevator.MoveToDataLevel()), wheels.DeployCoral(), flashingGreen.get()), // 3
            new SequentialCommandGroup(flashingRed.get(), drivetrain.PathfindToPose(DrivetrainConstants.BLUE_STAGING_POSES[3], 3, 3, 0), new ParallelCommandGroup(drivetrain.AlignCoralProfiled(), wrist.MoveToSelectedCoralDeployment(), elevator.MoveToDataLevel()), wheels.DeployCoral(), flashingGreen.get()), // 4
            new SequentialCommandGroup(flashingRed.get(), drivetrain.PathfindToPose(DrivetrainConstants.BLUE_STAGING_POSES[4], 3, 3, 0), new ParallelCommandGroup(drivetrain.AlignCoralProfiled(), wrist.MoveToSelectedCoralDeployment(), elevator.MoveToDataLevel()), wheels.DeployCoral(), flashingGreen.get()), // 5
            new SequentialCommandGroup(flashingRed.get(), drivetrain.PathfindToPose(DrivetrainConstants.BLUE_STAGING_POSES[5], 3, 3, 0), new ParallelCommandGroup(drivetrain.AlignCoralProfiled(), wrist.MoveToSelectedCoralDeployment(), elevator.MoveToDataLevel()), wheels.DeployCoral(), flashingGreen.get())  // 6 
        };

        this.RED_CORAL_MOD_COMMANDS = new Command[]
        {
            new SequentialCommandGroup(flashingRed.get(), drivetrain.PathfindToPose(DrivetrainConstants.RED_STAGING_POSES[0], 3, 3, 0), new ParallelCommandGroup(drivetrain.AlignCoralProfiled(), wrist.MoveToSelectedCoralDeployment(), elevator.MoveToDataLevel()), wheels.DeployCoral(), flashingGreen.get()), // 1
            new SequentialCommandGroup(flashingRed.get(), drivetrain.PathfindToPose(DrivetrainConstants.RED_STAGING_POSES[1], 3, 3, 0), new ParallelCommandGroup(drivetrain.AlignCoralProfiled(), wrist.MoveToSelectedCoralDeployment(), elevator.MoveToDataLevel()), wheels.DeployCoral(), flashingGreen.get()), // 2
            new SequentialCommandGroup(flashingRed.get(), drivetrain.PathfindToPose(DrivetrainConstants.RED_STAGING_POSES[2], 3, 3, 0), new ParallelCommandGroup(drivetrain.AlignCoralProfiled(), wrist.MoveToSelectedCoralDeployment(), elevator.MoveToDataLevel()), wheels.DeployCoral(), flashingGreen.get()), // 3
            new SequentialCommandGroup(flashingRed.get(), drivetrain.PathfindToPose(DrivetrainConstants.RED_STAGING_POSES[3], 3, 3, 0), new ParallelCommandGroup(drivetrain.AlignCoralProfiled(), wrist.MoveToSelectedCoralDeployment(), elevator.MoveToDataLevel()), wheels.DeployCoral(), flashingGreen.get()), // 4
            new SequentialCommandGroup(flashingRed.get(), drivetrain.PathfindToPose(DrivetrainConstants.RED_STAGING_POSES[4], 3, 3, 0), new ParallelCommandGroup(drivetrain.AlignCoralProfiled(), wrist.MoveToSelectedCoralDeployment(), elevator.MoveToDataLevel()), wheels.DeployCoral(), flashingGreen.get()), // 5
            new SequentialCommandGroup(flashingRed.get(), drivetrain.PathfindToPose(DrivetrainConstants.RED_STAGING_POSES[5], 3, 3, 0), new ParallelCommandGroup(drivetrain.AlignCoralProfiled(), wrist.MoveToSelectedCoralDeployment(), elevator.MoveToDataLevel()), wheels.DeployCoral(), flashingGreen.get())  // 6 
        };
    }
}
