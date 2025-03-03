package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ActuatorSubsystem;
import frc.robot.subsystems.OurAlgaeSubsystem;
import frc.robot.subsystems.OurAlgaeSubsystem.Setpoint;

public class SemiAuto {

      /* */
  /* Game action commands */
  public static Command runAlgaePickupLowerReefCommand(OurAlgaeSubsystem algae) {
    return new SequentialCommandGroup(
        algae.setSetpointCommand(Setpoint.kAlgaePickupLowerReef),
        new InstantCommand(() -> algae.moveToSetpoint())
       );
  }

  public static Command runAlgaePickupHigherReefCommand(OurAlgaeSubsystem algae) {
    return new SequentialCommandGroup(
        algae.setSetpointCommand(Setpoint.kAlgaePickupHigherReef),
        new InstantCommand(() -> algae.moveToSetpoint())
       );
  }
  public static Command runGroundPickupCommand(OurAlgaeSubsystem algae) {
    return new SequentialCommandGroup(
        algae.setSetpointCommand(Setpoint.kGroundPickup),
        new InstantCommand(() -> algae.moveToSetpoint())
       );
  }
  public static Command runShootAlgaeNetCommand(OurAlgaeSubsystem algae) {
    return new SequentialCommandGroup(
        algae.setSetpointCommand(Setpoint.kShootAlgaeNet),
        new InstantCommand(() -> algae.moveToSetpoint())
       );
  }

  public static Command runSideSlotShootCommand(OurAlgaeSubsystem algae, ActuatorSubsystem actuator) {
    return new SequentialCommandGroup(
        algae.setSetpointCommand(Setpoint.kSideSlotShoot),
        new InstantCommand(() -> algae.moveToSetpoint()),
        new InstantCommand(() -> actuator.markPosition()),
        new InstantCommand(() -> actuator.setInMotion(-1)).until(() -> actuator.isReachedSetpoint(-1))
       );
  }
}
