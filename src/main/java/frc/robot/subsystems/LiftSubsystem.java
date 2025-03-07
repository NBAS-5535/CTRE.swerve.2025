// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.AlgaeSubsystemConstants;

public class LiftSubsystem extends SubsystemBase {
  /** Creates a new LiftSubsystem. */
  private SparkMax liftMotor =
      new SparkMax(LiftConstants.kLiftMotorCanId, MotorType.kBrushless);
  private SparkClosedLoopController liftController = liftMotor.getClosedLoopController();
  private RelativeEncoder liftEncoder = liftMotor.getEncoder();

  private double initialPosition;

  public LiftSubsystem() {
    /* refer to Config.java
    SparkMaxConfig liftConfig = new SparkMaxConfig();

    liftConfig
        .smartCurrentLimit(LiftConstants.kLiftCurrentLimit)
        .closedLoopRampRate(LiftConstants.kLiftRampRate)
        .closedLoop
        .pid(LiftConstants.kLiftKp, LiftConstants.kLiftKi, LiftConstants.kLiftKd)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .outputRange(-1, 1);
        liftConfig.idleMode(IdleMode.kBrake);
    */

    liftMotor.configure(
        Configs.LiftSubsystem.liftConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Zero lift encoder on initialization
    this.setPosition(0);

    // sets max revolutions in Constants.java as reference -> moves the lift to that position (i.e., upright) 
    //liftController.setReference(LiftConstants.kSetPointInRevolutions, ControlType.kPosition);
    // set reference as the fully-retracted position
    liftController.setReference(0, ControlType.kPosition);

    initialPosition = liftEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //setInMotion();
    SmartDashboard.putNumber("Lift Position", liftEncoder.getPosition());
  }

  public void setPosition(double position){
    liftEncoder.setPosition(position);
  }
  
  public double getPosition(){
    return liftEncoder.getPosition();
  }

  public void setInMotion(int direction) {
    liftMotor.set(direction * LiftConstants.kSpeed);
    SmartDashboard.putNumber("Lift Speed", LiftConstants.kSpeed);
  }

  public void stopMotor() {
    liftMotor.set(0.);
  }

  /**
   * mark the current position of the lift
   *
   * @param none
   * @return {@link edu.wpi.first.wpilibj2.command.Command}
   */
  public Command markPositionCommand() {
    //initialPosition = getPosition();
    //SmartDashboard.putNumber("Lift Init", initialPosition);
    return this.runOnce( () -> this.markPosition());
  }

  // plain-vanilla marking
  public void markPosition() {
    initialPosition = getPosition();
    SmartDashboard.putNumber("Lift Init", initialPosition);
  }

  /**
   * check if the lift reached setpoint
   *
   * @param none 
   * @return true if setpoint is reached
   */
  /** Run the control loop to reach and maintain the setpoint from the preferences. */
  public boolean isReachedSetpoint(int direction) {
    double currentPosition = liftEncoder.getPosition();
    double [] temp = {initialPosition, currentPosition};
    SmartDashboard.putNumberArray("Lift Positions", temp);
    boolean condition = direction * currentPosition >= direction * initialPosition + LiftConstants.kSetPointInRevolutions;
    if ( condition ) {
      stopMotor();
      return true;
    } else {
      return false;
    }
  }

  /**
   * Command to run the elevator motor. 
   * Intended to step through to adjust proper setpoints for elevator heights
   * When the command is interrupted, e.g. the button is released, the motor will stop.
   */
  public Command runLiftUpCommand() {
    return this.startEnd(
        () -> this.setLiftPower(AlgaeSubsystemConstants.ElevatorSetpointTestSpeed), 
        () -> this.setLiftPower(0.0));
  }

  public Command runLiftDownCommand() {
    return this.startEnd(
        () -> this.setLiftPower((-1) * AlgaeSubsystemConstants.ElevatorSetpointTestSpeed), 
        () -> this.setLiftPower(0.0));
  }

  /** Set elevator motor power in the range of [-1, 1]. - TEST Purpose: step through */
  private void setLiftPower(double power) {
    liftMotor.set(power);
  }
}
