// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Pigeon2GyroSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class InstantCommandMarkGyroAngle extends InstantCommand {
    private final Pigeon2GyroSubsystem m_pidgy;

  public InstantCommandMarkGyroAngle(Pigeon2GyroSubsystem pigeon) {
    this.m_pidgy = pigeon; 
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pigeon);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pidgy.setAngleMarker();
  }
}
