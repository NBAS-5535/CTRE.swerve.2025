// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Pigeon2GyroSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OperatorFriendlyCommands extends Command {
  /** Creates a new OperatorFriendlyCommands. */
  private double m_initialAngle;
  private double continousAngle;

  private final CommandSwerveDrivetrain m_swerve;
  private final Pigeon2GyroSubsystem m_pidgy;

  public OperatorFriendlyCommands(CommandSwerveDrivetrain swerve, Pigeon2GyroSubsystem pigeon) {
    this.m_swerve = swerve;
    this.m_pidgy = pigeon;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_initialAngle = m_pidgy.getHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerve.sysIdRotate(Direction.kForward).withTimeout(10);
    continousAngle = m_pidgy.getHeading();
    SmartDashboard.putNumber("Current Angle", continousAngle);
    SmartDashboard.putNumber("Diff Angle", continousAngle - m_initialAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return angleDiffReached();
  }
  
  private Boolean angleDiffReached() {
    return Math.abs(continousAngle) - Math.abs(m_initialAngle) >= 90.;
  }
}
