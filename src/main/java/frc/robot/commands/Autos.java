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
import frc.robot.subsystems.Pigeon2GyroSubsystem;
import frc.robot.subsystems.ActuatorSubsystem;
import frc.robot.subsystems.ActuatorSubsystem.ActuatorSetpoints;
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
    return swerve.sysIdDynamic(direction).withTimeout(2);
  }

  /* move from line to reef; 
   * move actuator straight;
   * drop corral;
   * move to low reef seeting
   * pick up algae
   * move back
   * rotate 90deg
   * move to barge net
   * rotate 90deg
   * move towarda barge
   * move algae shoot setting
   * eject algae
   */
  public static Command midlineStartCommandPedantic(CommandSwerveDrivetrain swerve, 
                                            Pigeon2GyroSubsystem gyro, 
                                            AlgaeSubsystem algae,
                                            ActuatorSubsystem actuator) {
    Command tempCommand;
    tempCommand = new SequentialCommandGroup(
      //move forward
      new SequentialCommandGroup(
                //new InstantCommandMarkGyroPose(drivetrain),
                new InstantCommand(() -> swerve.setCurrentPose()),
                swerve.sysIdDynamic(Direction.kForward).until(() -> swerve.isDesiredPoseReached(1.5))
                ),
      // add corraldrop and low reef pickup
      new SequentialCommandGroup(
                actuator.setSetpointCommand(ActuatorSetpoints.kSetPointInRevolutions),
                algae.setSetpointCommand(AlgaeSubsystem.Setpoint.kCorralDrop),
                ManualCommands.runIntakeCommand(algae)
                ),
      // move back
      new SequentialCommandGroup(
                  //new InstantCommandMarkGyroPose(drivetrain),
                  new InstantCommand(() -> swerve.setCurrentPose()),
                  swerve.sysIdDynamic(Direction.kForward).until(() -> swerve.isDesiredPoseReached(0.6))
                  ),
      // rotate set angel tol to 10deg??
      new SequentialCommandGroup(
                new InstantCommand(() -> gyro.setAngleMarker()),
                swerve.sysIdRotate(Direction.kForward).until(() -> gyro.isAngleDiffReached(swerve, 90.))
                ),
      //move to wards barge
      new SequentialCommandGroup(
                //new InstantCommandMarkGyroPose(drivetrain),
                new InstantCommand(() -> swerve.setCurrentPose()),
                swerve.sysIdDynamic(Direction.kForward).until(() -> swerve.isDesiredPoseReached(2.0))
                ),
      // rotate
      new SequentialCommandGroup(
                new InstantCommand(() -> gyro.setAngleMarker()),
                swerve.sysIdRotate(Direction.kForward).until(() -> gyro.isAngleDiffReached(swerve, 90))
                ),
      new SequentialCommandGroup(
                //new InstantCommandMarkGyroPose(drivetrain),
                new InstantCommand(() -> swerve.setCurrentPose()),
                swerve.sysIdDynamic(Direction.kForward).until(() -> swerve.isDesiredPoseReached(0.8))
                )
      // add highreef setting and shooting

    );
    return tempCommand;
  }

  /* action to be stitched together if desired */
  /* go distance: in encoder value */
  public static Command moveByDistance(CommandSwerveDrivetrain swerve, double encoderPosition) {
    Direction direction = Direction.kForward;
    if ( encoderPosition < 0. ) {
      direction = Direction.kReverse;
    }
    return new SequentialCommandGroup(
      //new InstantCommandMarkGyroPose(drivetrain),
      new InstantCommand(() -> swerve.setCurrentPose()),
      swerve.sysIdDynamic(direction).until(() -> swerve.isDesiredPoseReached(encoderPosition))
      );
  }

  /* rotate by angle using Pigeon info */
  public static Command rotateByAngleInDegrees(CommandSwerveDrivetrain swerve, 
                                               Pigeon2GyroSubsystem gyro,
                                               double angle) {
    return new SequentialCommandGroup(
      new InstantCommand(() -> gyro.setAngleMarker()),
      swerve.sysIdRotate(Direction.kForward).until(() -> gyro.isAngleDiffReached(swerve, angle))
      );
  }

  /* rotate by angle using Pigeon info */
  public static Command rotateByTime(CommandSwerveDrivetrain swerve, 
                                     Direction direction) {
    return swerve.sysIdRotate(direction);
  }

  /* position for corral drop */
  public static Command dropCorralOnLowerLevel(AlgaeSubsystem algae,
                                               ActuatorSubsystem actuator) {
    return new SequentialCommandGroup(
      algae.setSetpointCommand(Setpoint.kCorralDrop),
      actuator.setSetpointCommand(ActuatorSetpoints.kAlgaeNetShootSetPoint),
      algae.setSetpointCommand(Setpoint.kShootCorralDrop)//,
      //algae.runIntakeCommand()//.withTimeout(1.) // to eject the corral
      );
  }

  /* activate intake and move to low algae pickup */
  public static Command pickupAlgaeFromLowReef(AlgaeSubsystem algae) {
    return new SequentialCommandGroup(
      algae.setSetpointCommand(Setpoint.kAlgaePickupLowerReef),
      algae.runIntakeCommand().withTimeout(1.) // retrieve algae
    );
  }

  /* activate intake and move to low algae pickup */
  public static Command pickupAlgaeFromHighReef(AlgaeSubsystem algae) {
    return new SequentialCommandGroup(
      algae.setSetpointCommand(Setpoint.kAlgaePickupHigherReef),
      algae.runIntakeCommand().withTimeout(1.) // retrieve algae
    );
  }

  /* move to algae shoot and activate intake in reverse */
  public static Command shootAlgaeIntoNet(AlgaeSubsystem algae) {
    return new SequentialCommandGroup(
      algae.setSetpointCommand(Setpoint.kShootAlgaeNet),
      algae.reverseIntakeCommand().withTimeout(1.)
    );
  }

  /* COMPOSITE Commands */
  /* midline start commmand reimagined */
  public static Command midlineStartCommand(CommandSwerveDrivetrain swerve, 
                                    Pigeon2GyroSubsystem gyro, 
                                    AlgaeSubsystem algae,
                                    ActuatorSubsystem actuator) {
    double timeout = 10.; // seconds between commands
    double motionTime = 0.6; // seconds to rotate for 90deg <---------
    Command tempCommand = new SequentialCommandGroup(
      moveByDistance(swerve, 2.1),            //move forward 88"
     // Commands.waitSeconds(timeout),
      //Commands.waitSeconds(timeout),
      
      dropCorralOnLowerLevel(algae, actuator),                //drop corral
      Commands.waitSeconds(1.),
      algae.runIntakeCommand().withTimeout(1.5), // to eject the corral
      
      moveByDistance(swerve, 0.3),            //move closer for pickup
      algae.setSetpointCommand(Setpoint.kAlgaePickupHigherReef),
      Commands.waitSeconds(2.),
      algae.runIntakeCommand().withTimeout(1.5), // to get algae
      Commands.waitSeconds(2.),
      moveByDistance(swerve, -0.6)            //move back to rotate
      /*
      Commands.waitSeconds(3*timeout),
      
      moveByDistance(swerve, 0.2),            //move closer for pickup
      Commands.waitSeconds(timeout),

      pickupAlgaeFromHighReef(algae),                          //get algae
      Commands.waitSeconds(timeout),
      
      moveByDistance(swerve, 0.6),            //move back to rotate
      Commands.waitSeconds(timeout),
      
      //rotateByAngleInDegrees(swerve, gyro, -90.),        //rotate 90deg
      rotateByTime(swerve, Direction.kReverse).withTimeout(motionTime),
      Commands.waitSeconds(timeout),
      
      moveByDistance(swerve, 2.0),            //move to algae net/barge ~100"
      Commands.waitSeconds(timeout),
      
      //rotateByAngleInDegrees(swerve, gyro, -90.),        //rotate 90deg towards algaenet
      rotateByTime(swerve, Direction.kReverse).withTimeout(motionTime),
      Commands.waitSeconds(timeout),
      
      moveByDistance(swerve, 0.8),            //move closer to algae net/barge
      Commands.waitSeconds(timeout),
      
      shootAlgaeIntoNet(algae)                                //shoot algae into net
      */
    );
    return tempCommand;
  }

  /* start in front of the Blue or Red barge */
  public static Command algaenetSideStart(CommandSwerveDrivetrain swerve, 
                                          Pigeon2GyroSubsystem gyro, 
                                          AlgaeSubsystem algae,
                                          ActuatorSubsystem actuator) {
    double timeout = 10.; // seconds between commands
    double motionTime = 0.333; // seconds to rotate for 90deg <--------- change to a value for ANGLE = 45-60deg value
    Command tempCommand = new SequentialCommandGroup(
      moveByDistance(swerve, 0.2),            //move closer for pickup 30.3"
      Commands.waitSeconds(timeout),

      rotateByTime(swerve, Direction.kReverse).withTimeout(motionTime), //rotate toward reef side by ANGLE
      moveByDistance(swerve, 2.5),            //move forward 115.5"
      Commands.waitSeconds(timeout),
      
      dropCorralOnLowerLevel(algae, actuator),                //drop corral
      Commands.waitSeconds(timeout),
      
      moveByDistance(swerve, 0.2),            //move closer for pickup
      Commands.waitSeconds(timeout),

      pickupAlgaeFromHighReef(algae),                          //get algae
      Commands.waitSeconds(timeout),
      
      moveByDistance(swerve, 0.6),            //move back to rotate
      Commands.waitSeconds(timeout),
      
      //rotateByAngleInDegrees(swerve, gyro, -180.),        //rotate 180deg
      rotateByTime(swerve, Direction.kReverse).withTimeout(motionTime), // 180deg
      Commands.waitSeconds(timeout),
      
      moveByDistance(swerve, 2.0),            //move to algae net/barge
      Commands.waitSeconds(timeout),
      //rotateByAngleInDegrees(swerve, gyro, -180.),        //rotate 180deg towards algaenet
      
      rotateByTime(swerve, Direction.kForward).withTimeout(motionTime), // rotate to face the net by ANGLE
      Commands.waitSeconds(timeout),

      shootAlgaeIntoNet(algae)                                //shoot algae into net
    );
    return tempCommand;
}
}
