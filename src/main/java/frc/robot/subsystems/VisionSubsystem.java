// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  private static NetworkTable m_table = NetworkTableInstance.getDefault().getTable("limelight");

  public VisionSubsystem() {

  }

  public boolean hasTarget() {
        return m_table.getEntry("tv").getDouble(0) == 1;
    }

    public double getTx() {
        return m_table.getEntry("tx").getDouble(0);
    }

    public double getTy() {
        return m_table.getEntry("ty").getDouble(0);
    }

    public double getTa() {
        return m_table.getEntry("ta").getDouble(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Limelight Has Target", hasTarget());
        SmartDashboard.putNumber("Limelight X Offset", getTx());
        SmartDashboard.putNumber("Limelight Y Offset", getTy());
        SmartDashboard.putNumber("Limelight Area", getTa());
    }
}
