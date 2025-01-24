// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2Configurator;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pigen2GyroSubsystem extends SubsystemBase {
  /** Creates a new Pigen2GyroSubsystem. */
  private final Pigeon2 m_pigeon2 = new Pigeon2(40, "rio");
  private final Pigeon2SimState m_pigeon2SimState = m_pigeon2.getSimState();

  public Pigen2GyroSubsystem() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
     * Initialize Pigeon2 device from the configurator object
     * 
     * @param cfg Configurator of the Pigeon2 device
     */
    private void initializePigeon2(Pigeon2Configurator cfg) {
        var toApply = new Pigeon2Configuration();

        /*
         * User can change configs if they want, or leave this blank for factory-default
         */

        cfg.apply(toApply);

        /* And initialize yaw to 0 */
        cfg.setYaw(48);
    }
}
