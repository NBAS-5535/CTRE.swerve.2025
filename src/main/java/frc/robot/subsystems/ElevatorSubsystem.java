// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Set;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */

  // for sim only
  // private final DCMotor m_elevatorGearbox = DCMotor.getNEO(1);

  private final SparkFlex m_motor   = new SparkFlex(ElevatorConstants.kElevatorCANId, MotorType.kBrushless);
  private final RelativeEncoder m_encoder  = m_motor.getEncoder();
  
  private final ProfiledPIDController m_controller = new ProfiledPIDController(ElevatorConstants.kElevatorKp,
                                                                               ElevatorConstants.kElevatorKi,
                                                                               ElevatorConstants.kElevatorKd,
                                                                               new Constraints(ElevatorConstants.kMaxVelocity,
                                                                                               ElevatorConstants.kMaxAcceleration));
 
  public ElevatorSubsystem() {
    SparkFlexConfig config = new SparkFlexConfig();
    config
        .smartCurrentLimit(ElevatorConstants.kElevatorCurrentLimit)
        .closedLoopRampRate(ElevatorConstants.kElevatorRampRate)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .outputRange(-1, 1);
    m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

  }
  

  public enum ElevatorState {
    NONE,
    STOW,
    L2,
    L3,
    L4,
    A1,
    A2
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
  public void reachGoal(double goal)
  {
    /*
    double voltsOut = MathUtil.clamp(
        m_controller.calculate(getHeightMeters(), goal) +
        m_feedforward.calculateWithVelocities(getVelocityMetersPerSecond(),
                                              m_controller.getSetpoint().velocity), -7, 7);
    m_motor.setVoltage(voltsOut);
    */
  }

  /**
   * Set the goal of the elevator
   *
   * @param goal Goal in meters
   * @return {@link edu.wpi.first.wpilibj2.command.Command}
   */
  public Command setGoal(double goal)
  {
    return run(() -> reachGoal(goal));
  }

  /**
   * Stop the control loop and motor output.
   */
  public void stop()
  {
    m_motor.set(0.0);
  }
}
