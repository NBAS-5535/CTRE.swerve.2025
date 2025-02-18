// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.OperatorFriendlyCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ActuatorSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Pigeon2GyroSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem_Test;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.SysIdSwerveTranslation goForward = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    /* gyro */
    public final Pigeon2 pigeon2 = drivetrain.getPigeon2();
    public final Pigeon2GyroSubsystem pigeon2Subsystem = new Pigeon2GyroSubsystem(pigeon2);

    /* Limelight */
    /* initial attempt */
    //public final VisionSubsystem_Test limelight = new VisionSubsystem_Test();
    private final VisionSubsystem m_vision = new VisionSubsystem();

    /* 2025 game related subsystems */
    /* actuator to move the levator to game start position */
    public final ActuatorSubsystem m_actuator = new ActuatorSubsystem();

    /* elevator subsystem */
    //public final ElevatorSubsystem m_elevator = new ElevatorSubsystem();

    /************** Ctor */
    public RobotContainer() {
        //autoChooser = AutoBuilder.buildAutoChooser("TestPath");
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);
        
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Rotate by a specific angle
        double tempAngle = Math.PI / 2.;
        //joystick.x().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(tempAngle))));


        // drive at a constant speed
        joystick.y().whileTrue(drivetrain.applyRequest(() -> goForward.withVolts(0.2 * MaxSpeed)));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward).withTimeout(2));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // prefixed movement in +/- X-direction
        /*
        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );
        */

        // let's try rotation
        //joystick.povUp().onTrue(drivetrain.sysIdRotate(Direction.kForward));
        //joystick.povUp().whileTrue(drivetrain.sysIdRotate(Direction.kForward));
        // Rotate by 90deg using a fixed speed and time
        // joystick.povUp().onTrue(drivetrain.sysIdRotate(Direction.kForward).withTimeout(0.67));

        // point forward
        /*
        joystick.povRight().onTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(tempAngle))
        ));
        */
        /* Rotate by 90deg in-place using a fixed rotational speed
        joystick.povLeft().whileTrue(drivetrain.applyRequest(() ->
            drive.withVelocityX(0) // Drive forward with negative Y (forward)
            .withVelocityY(0) // Drive left with negative X (left)
            .withRotationalRate(tempAngle * 0.1 * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));
        */
        SmartDashboard.putNumber("Angle", tempAngle);
        SmartDashboard.putNumber("MaxAngularVelocity", MaxAngularRate);

        //joystick.povDown().whileTrue(new OperatorFriendlyCommands(drivetrain, pigeon2Subsystem));
        //pigeon2Subsystem.setAngleMarker();
        //SmartDashboard.putNumber("Reference Angle", pigeon2Subsystem.getHeading());
        /* rotate robot "gradually" until ~90deg is reached*/
        joystick.povDown().onTrue(new SequentialCommandGroup(
            //new InstantCommandMarkGyroAngle(pigeon2Subsystem),
            new InstantCommand(() -> pigeon2Subsystem.setAngleMarker()),
            //new OperatorFriendlyCommands(drivetrain, pigeon2Subsystem, "rotate"),
            //Commands.print("Reset the angles"),
            drivetrain.sysIdRotate(Direction.kForward).until(() -> pigeon2Subsystem.isAngleDiffReached()))
            );

        /* get robot Pode/location info */
        /* pedantic way */
        /*
        joystick.povRight().onTrue(new SequentialCommandGroup(
            new OperatorFriendlyCommands(drivetrain, pigeon2Subsystem, "pose"),
            drivetrain.sysIdDynamic(Direction.kForward).withTimeout(1),
            new OperatorFriendlyCommands(drivetrain, pigeon2Subsystem, "pose")
        ));
        */
        /* fancy way: 
         * Mark current position
         * Drive forward for 2 meters
        */
        joystick.povRight().onTrue(new SequentialCommandGroup(
            //new InstantCommandMarkGyroPose(drivetrain),
            new InstantCommand(() -> drivetrain.setCurrentPose()),
            drivetrain.sysIdDynamic(Direction.kForward).until(() -> drivetrain.isDesiredPoseReached(2.))
        ));

        // get vision-based distance
        //joystick.x().onTrue(new InstantCommand(() -> limelight.getDistanceToTarget()));
        /* onTrue: robot moves until the alignment is completed
        *  whileTrue: must press the button until the alignment is completed
        */
        //joystick.x().onTrue(new AlignCommand(drivetrain, m_vision, VisionConstants.testTagId));
        /* simulate a sequence:
         * align with AprilTag
         * Move forward by 2 meters
         */
        joystick.x().onTrue(new SequentialCommandGroup(
            new AlignCommand(drivetrain, m_vision, VisionConstants.testTagId),
            //drivetrain.applyRequest(() -> brake),
            drivetrain.sysIdDynamic(Direction.kForward).until(() -> drivetrain.isDesiredPoseReached(2.))
        ));

        // move the elevator to game position: direction =1
        joystick.povUp().whileTrue(new SequentialCommandGroup(
            m_actuator.markPositionCommand(),
            //new InstantCommand(() -> m_actuator.markPosition()),
            //new InstantCommand(() -> m_actuator.setInMotion(1)).until(() -> m_actuator.isReachedSetpoint(1))
            new InstantCommand(() -> m_actuator.setInMotion(1)).withTimeout(0.5))
        );
        // move elevator back to start position
        joystick.povCenter().onTrue(new SequentialCommandGroup(
            new InstantCommand(() -> m_actuator.markPosition()),
            //new InstantCommand(() -> m_actuator.setInMotion(-1)).until(() -> m_actuator.isReachedSetpoint(-1))
            new InstantCommand(() -> m_actuator.setInMotion(-1)).withTimeout(0.5))
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // some autonomous sequences
        String caseType = "auto"; //"manual";
        Command autoCommand = null;
        switch (caseType) {
            case "manual":
                autoCommand = Commands.sequence(
                    drivetrain.sysIdDynamic(Direction.kForward).withTimeout(0.5),
                    //drivetrain.applyRequest(() -> brake),
                    Commands.waitSeconds(3.0),
                    drivetrain.sysIdRotate(Direction.kForward).withTimeout(0.34),
                    Commands.waitSeconds(1.),
                    drivetrain.sysIdRotate(Direction.kForward).withTimeout(0.68),
                    Commands.waitSeconds(2.),
                    drivetrain.sysIdDynamic(Direction.kForward).withTimeout(0.5)
                );
                break;
            case "auto":
                autoCommand = Autos.moveRotateRestRepeat(drivetrain);
                break;
            case "path":
                /* Run the path selected from the auto chooser */
                //autoCommand = new PathPlannerAuto("FancyAutoPath"); //
                //autoCommand = autoChooser.getSelected();
                try{
                    // Load the path you want to follow using its name in the GUI
                    PathPlannerPath path = PathPlannerPath.fromPathFile("FancyAutoPath");

                    // Create a path following command using AutoBuilder. This will also trigger event markers.
                    autoCommand = AutoBuilder.followPath(path);
                } catch (Exception e) {
                    DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
                    autoCommand =  Commands.none();
                }
                break;
            default:
                autoCommand = Commands.print("No autonomous command configured");
        }
        return autoCommand;

    }

}
