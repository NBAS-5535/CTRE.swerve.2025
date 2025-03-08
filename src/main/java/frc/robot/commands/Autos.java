// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.AlgaeSubsystem.Setpoint;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Autos extends Command {
  /** Creates a new Autos. */
  public Autos() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  /* test */
  public static Command moveRotateRestRepeat(CommandSwerveDrivetrain swerve){
    Command autoCommand = Commands.sequence(
      swerve.sysIdDynamic(Direction.kForward).withTimeout(0.1),
            Commands.waitSeconds(5.0),
            swerve.sysIdRotate(Direction.kForward).withTimeout(0.65), // for 90deg, rotate for 0.333s at pi rad/s
            Commands.waitSeconds(5.),
            swerve.sysIdRotate(Direction.kForward).withTimeout(0.65),
            Commands.waitSeconds(5.),
            swerve.sysIdDynamic(Direction.kReverse).withTimeout(0.1)
        );
    return autoCommand;
  }

  /* Game scenarios */
  /* move off the start line by driving forward for 1 sec */
  public static Command moveOffTheLine(CommandSwerveDrivetrain swerve, Direction direction){
    return swerve.sysIdDynamic(direction).withTimeout(1);
  }


}
