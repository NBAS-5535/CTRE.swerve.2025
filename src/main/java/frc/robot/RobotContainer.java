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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ActuatorSubsystemConstants;
import frc.robot.Constants.AutonomousMenuConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.ActuatorCommandRev;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.ManualCommands;
import frc.robot.commands.OperatorFriendlyCommands;
import frc.robot.commands.SemiAuto;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ActuatorSubsystemRev;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ActuatorSubsystem;
import frc.robot.subsystems.ActuatorSubsystem.ActuatorSetpoints;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.AlgaeSubsystem.Setpoint;
import frc.robot.subsystems.Pigeon2GyroSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem_Test;
import frc.robot.subsystems.LiftSubsystem;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final double speedDeadBand = 0.08;
    private final double angleDeadBand = 0.1;
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * speedDeadBand).withRotationalDeadband(MaxAngularRate * angleDeadBand) // Add a deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.SysIdSwerveTranslation goForward = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
    //private final SwerveRequest.Fieldcentric
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController txbox = new CommandXboxController(1);

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
    public final boolean actuatorIsRev = false;
    public final ActuatorSubsystem m_actuator = new ActuatorSubsystem();
    //public final boolean actuatorIsRev = true;
    //public final ActuatorSubsystemRev m_actuator = new ActuatorSubsystemRev();


    /* elevator subsystem */
    //public final ElevatorSubsystem m_elevator = new ElevatorSubsystem();

    /** OurAlgaeSubsystem */
    private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();

    /** LiftSubsystem */
    private final LiftSubsystem m_liftSubsystem = new LiftSubsystem();

    /* */

    /* Lift subsystem */
    //private final LiftSubsystem m_liftSubsystem = new LiftSubsystem();

    /* autonomous dropdown menu */
    private SendableChooser<Command> dropDownChooser;// = new SendableChooser<>();
    private SendableChooser<String> autonomousChooser ;
    

    /************** Ctor */
    public RobotContainer() {
        //autoChooser = AutoBuilder.buildAutoChooser("TestPath");
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("PP Mode", autoChooser);

        /* dropdown autonomous menu 
        dropDownChooser = new SendableChooser<>();
        //dropDownChooser.setDefaultOption("Default Auto", drivetrain.sysIdDynamic(Direction.kForward).withTimeout(0.5));
        dropDownChooser.setDefaultOption("Default Auto", drivetrain.sysIdDynamic(Direction.kForward).withTimeout(0.5));
        dropDownChooser.addOption("Move Back", drivetrain.sysIdDynamic(Direction.kReverse).withTimeout(0.5));
        dropDownChooser.addOption("moveRotateRestRepeat", Autos.moveRotateRestRepeat(drivetrain));
        SmartDashboard.putData("Auto Menu", dropDownChooser);
        */

        /* autonomous position chooser */
        autonomousChooser = new SendableChooser<>();
        autonomousChooser.setDefaultOption("No Action", "none");
        autonomousChooser.addOption("DownBlue/Blue1", AutonomousMenuConstants.kDownBlue);
        autonomousChooser.addOption("CenterBlue/Blue2", AutonomousMenuConstants.kCenterBlue);
        autonomousChooser.addOption("UpBlue/Blue3", AutonomousMenuConstants.kUpBlue);
        autonomousChooser.addOption("DownRed/Red1", AutonomousMenuConstants.kDownRed);
        autonomousChooser.addOption("CenterRed/Red2", AutonomousMenuConstants.kCenterRed);
        autonomousChooser.addOption("UpRed/Red3", AutonomousMenuConstants.kUpRed);
        SmartDashboard.putData("AutonomousMenu", autonomousChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        /* This makes the current orientation of the robot X forward for field-centric maneuvers.  */
        // reset the field-centric heading on left bumper press
        joystick.a().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        /* */
        //joystick.rightBumper().whileTrue(drivetrain.applyRequest(() -> brake));

        /* Swerve DriveTrain */

        /* reused down below - check ourcoral
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));
        */ 
        
        // drive at a constant speed
        //joystick.y().whileTrue(drivetrain.applyRequest(() -> goForward.withVolts(0.2 * MaxSpeed)));


        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        boolean driveTest = false;
        if (driveTest) {
            joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));//.withTimeout(2));
            joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
            joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
            joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        } // end driveTest

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
        boolean rotationTest = true;
        if ( rotationTest ) {
            joystick.back().and(joystick.y()).onTrue(drivetrain.sysIdRotate(Direction.kForward).withTimeout(0.64));
            joystick.back().and(joystick.x()).onTrue(drivetrain.sysIdRotate(Direction.kReverse).withTimeout(0.63));
            joystick.start().and(joystick.y()).onTrue(drivetrain.sysIdRotate(Direction.kForward).withTimeout(0.62));
            joystick.start().and(joystick.x()).onTrue(drivetrain.sysIdRotate(Direction.kReverse).withTimeout(0.61));
            //joystick.x().whileTrue(drivetrain.sysIdRotate(Direction.kForward));
            //joystick.y().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
         
            //drivetrain.getModule(0).getDriveMotor().getPosition().getValue();
        } // end driveTest

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
        // Rotate by a specific angle
        double tempAngle = Math.PI / 2.;
        SmartDashboard.putNumber("Angle", tempAngle);
        SmartDashboard.putNumber("MaxAngularVelocity", MaxAngularRate);

        /* rotate by setting a marker and turning the robot until 90deg is reached */
        if (true) {
            //joystick.povDown().whileTrue(new OperatorFriendlyCommands(drivetrain, pigeon2Subsystem));
            //pigeon2Subsystem.setAngleMarker();
            //SmartDashboard.putNumber("Reference Angle", pigeon2Subsystem.getHeading());
            /* rotate robot "gradually" until ~90deg is reached*/
            joystick.b().onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> pigeon2Subsystem.setAngleMarker()),
                drivetrain.sysIdRotate(Direction.kForward).until(() -> pigeon2Subsystem.isAngleDiffReached(drivetrain, 90.)))
                );
            
        } // end driveTest

        /* get robot Pose/location info */
        if (true) {
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
            txbox.povDown().onTrue(new SequentialCommandGroup(
                //new InstantCommandMarkGyroPose(drivetrain),
                new InstantCommand(() -> drivetrain.setCurrentPose()),
                drivetrain.sysIdDynamic(Direction.kForward).until(() -> drivetrain.isDesiredPoseReached(1.5))
            ));
            
        } // end driveTest

        /* Vision Subsystem */
        boolean visionTest = false;
        if (visionTest) {
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
            /* Testing Closest versus a specific AprilTag */
            boolean closestTag = true;
            // Default: a specific tag number
            int testTagId = VisionConstants.testTagId;
            if ( closestTag) {
                testTagId = 0;
            }
            
            joystick.x().onTrue(new SequentialCommandGroup(
                new AlignCommand(drivetrain, m_vision, testTagId),
                //drivetrain.applyRequest(() -> brake),
                drivetrain.sysIdDynamic(Direction.kForward).until(() -> drivetrain.isDesiredPoseReached(1.))
            ));
        } // end visionTest

        /* Actuator Subsystem */
        boolean actuatorTest = true;
        if (actuatorTest) {
            
            // move the elevator to game position: direction =1
            if ( actuatorIsRev ) {
                /*
                joystick.leftBumper().onTrue(
                    new SequentialCommandGroup(
                        new ActuatorCommandRev(m_actuator, 1, ActuatorSubsystemConstants.kSetPointInRevolutions),
                        new InstantCommand(() -> m_actuator.stopMotor()))
                );
                // move elevator back to start position
                joystick.rightBumper().onTrue(
                    new SequentialCommandGroup(
                        new ActuatorCommandRev(m_actuator, -1, ActuatorSubsystemConstants.kSetPointInRevolutions),
                        new InstantCommand(() -> m_actuator.stopMotor()))
               );
                // move to an intermediate point
                txbox.povRight().onTrue(
                    new SequentialCommandGroup(
                        new ActuatorCommandRev(m_actuator, -1, ActuatorSubsystemConstants.kIntermediateSetPoint),
                        new InstantCommand(() -> m_actuator.stopMotor()))
                );
                */
            } else {
                /* uncomment if needed */
                joystick.povRight().onTrue(
                    m_actuator.setSetpointCommand(ActuatorSetpoints.kSetPointInRevolutions)
                );
                // move elevator back to start position
                joystick.povLeft().onTrue(
                    m_actuator.setSetpointCommand(ActuatorSetpoints.kBase)
                );
                // move to an intermediate point
                //joystick.start().onTrue(
                //    m_actuator.setSetpointCommand(ActuatorSetpoints.kAlgaeNetShootSetPoint)
                //);
                /**/
            }
        } // end actuator test buttons

        /* Algae Subsystem */
        boolean algaeSubsystemTuning = false;
        if (algaeSubsystemTuning) {
            /* Try gradually moving the elevator to determine operational heights */
            // A -> Run elevator UP

            joystick.a().whileTrue(ManualCommands.runElevatorUpCommand(m_algaeSubsystem));
            // B -> Run elevator DOWN
            joystick.b().whileTrue(ManualCommands.runElevatorDownCommand(m_algaeSubsystem));

            /* Try gradually moving the arm to determine operational heights */
            // povUp -> Run arm UP
            joystick.povUp().whileTrue(ManualCommands.runArmUpCommand(m_algaeSubsystem));
            // povDown -> Run arm DOWN
            joystick.povDown().whileTrue(ManualCommands.runArmDownCommand(m_algaeSubsystem));
        } // end of elevator/arm test buttons

        /* quick test code 

            // A -> Run elevator UP
            joystick.a().whileTrue(ManualCommands.runElevatorUpCommand(m_algaeSubsystem));
            // B -> Run elevator DOWN
            joystick.b().whileTrue(ManualCommands.runElevatorDownCommand(m_algaeSubsystem));

            // start -> Run arm UP
            joystick.start().whileTrue(ManualCommands.runArmUpCommand(m_algaeSubsystem));
            // back -> Run arm DOWN
            joystick.back().whileTrue(ManualCommands.runArmDownCommand(m_algaeSubsystem));
        */
        
        /* !!!!!! Button definitiotions for COMPETITION !!!!!!!!  */
        /* run intake motor in suck-in and push-out modes */
        // povRight -> Run tube intake
        joystick.rightBumper().whileTrue(ManualCommands.runIntakeCommand(m_algaeSubsystem));

        // povLeft -> Run tube intake in reverse
        joystick.leftBumper().whileTrue(ManualCommands.reverseIntakeCommand(m_algaeSubsystem));

        /* run lift motor in suck-in and push-out modes */
        // povRight -> Run tube intake
        joystick.y().whileTrue(m_liftSubsystem.runLiftUpCommand());

        // povLeft -> Run tube intake in reverse
        //joystick.y().whileTrue(m_liftSubsystem.runLiftDownCommand());
        
        // move elevator/arm to their respective positions
        txbox
            .a()
                .onTrue(
                    //m_algaeSubsystem.setSetpointCommand(Setpoint.kAlgaePickupLowerReef)
                    new SequentialCommandGroup(
                            m_algaeSubsystem.setSetpointCommand(Setpoint.kClearWires),
                            m_algaeSubsystem.setSetpointCommand(Setpoint.kAlgaePickupLowerReef),
                            m_actuator.setSetpointCommand(ActuatorSetpoints.kSetPointInRevolutions)
                    )    
        );

        txbox
            .b()
                .onTrue(
                    new SequentialCommandGroup(
                        m_algaeSubsystem.setSetpointCommand(Setpoint.kClearWires),
                        m_algaeSubsystem.setSetpointCommand(Setpoint.kAlgaePickupHigherReef),
                        Commands.waitSeconds(1.),
                        m_actuator.setSetpointCommand(ActuatorSetpoints.kSetPointInRevolutions)
                    )
                    //m_algaeSubsystem.setSetpointCommand(Setpoint.kAlgaePickupHigherReef)                   
        );
        
        txbox
            .x()
                .onTrue(
                    m_algaeSubsystem.setSetpointCommand(Setpoint.kGroundPickup)
                    /*
                    new ParallelCommandGroup(
                        m_algaeSubsystem.setSetpointCommand(Setpoint.kGroundPickup)//,
                        m_actuator.setSetpointCommand(ActuatorSetpoints.kSetPointInRevolutions)
                    )
                        */
        );

        txbox
            .y()
                .onTrue(
                    //m_algaeSubsystem.setSetpointCommand(Setpoint.kShootAlgaeNet)
                    new ParallelCommandGroup(
                        m_algaeSubsystem.setSetpointCommand(Setpoint.kShootAlgaeNet),
                        m_actuator.setSetpointCommand(ActuatorSetpoints.kAlgaeNetShootSetPoint)
                    )
        );

        //
        txbox
            .povUp()
                .onTrue(
                    new SequentialCommandGroup(
                        //m_algaeSubsystem.setSetpointCommand(Setpoint.kMoveWithBall),
                        m_algaeSubsystem.setSetpointCommand(Setpoint.kSideSlotShoot),
                        m_actuator.setSetpointCommand(ActuatorSetpoints.kSetPointInRevolutions)
                    )    
        );

        /* corral drop: most likely an autonomous functionality */
        txbox
            .povDown()
                .onTrue(
                    new SequentialCommandGroup(
                        m_algaeSubsystem.setSetpointCommand(Setpoint.kCorralDrop),
                        m_actuator.setSetpointCommand(ActuatorSetpoints.kAlgaeNetShootSetPoint),
                        m_algaeSubsystem.setSetpointCommand(Setpoint.kShootCorralDrop)
                    )
        );

        /* move with ball */
        txbox
            .rightBumper()
                .onTrue(
                new SequentialCommandGroup(
                        m_algaeSubsystem.setSetpointCommand(Setpoint.kMoveWithBall),
                        m_actuator.setSetpointCommand(ActuatorSetpoints.kSetPointInRevolutions)
                    )
        );
            
                
    
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // some autonomous sequences
        String caseType = "menu"; //"manual";
        Command autoCommand = null;
        String menuItem = "";
        String chosenItem = "";
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
            case "menu":
                //autoCommand = dropDownChooser.getSelected();
                /* IMPORTANT: Designations are wrt PathPlanner layout: 
                *             Blue on the left / Red on the right 
                *             Blue AlgeNet/barge is BlueUP
                *             Red AlgeNet/barge is RedDOWN
                */
                menuItem = autonomousChooser.getSelected();
                chosenItem = "NO ACTION";
                switch (menuItem){
                    case AutonomousMenuConstants.kDownBlue:
                        chosenItem = "BlueDown_1";
                        autoCommand = Autos.moveOffTheLine(drivetrain, Direction.kForward);
                        break;
                    case AutonomousMenuConstants.kCenterBlue:
                        chosenItem = "BlueCenter_2";
                        autoCommand = Autos.midlineStartCommand(drivetrain, pigeon2Subsystem, m_algaeSubsystem, m_actuator);
                        break;
                    case AutonomousMenuConstants.kUpBlue:
                        chosenItem = "BlueUp_3";
                        autoCommand = Autos.algaenetSideStart(drivetrain, pigeon2Subsystem, m_algaeSubsystem, m_actuator);
                        break;
                    case AutonomousMenuConstants.kDownRed:
                        chosenItem = "RedDown_4";
                        autoCommand = Autos.algaenetSideStart(drivetrain, pigeon2Subsystem, m_algaeSubsystem, m_actuator);
                        break;
                    case AutonomousMenuConstants.kCenterRed:
                        chosenItem = "RedCenter_5";
                        autoCommand = Autos.midlineStartCommand(drivetrain, pigeon2Subsystem, m_algaeSubsystem, m_actuator);
                        break;
                    case AutonomousMenuConstants.kUpRed:
                        chosenItem = "RedUp_6";
                        autoCommand = Autos.moveOffTheLine(drivetrain, Direction.kForward);
                        break; 
                    default:
                        chosenItem = "Nothing"; 
                }
                SmartDashboard.putString("Menu-Pick", chosenItem);
                break;
            case "path":
                /* Run the path selected from the auto chooser */
                //autoCommand = new PathPlannerAuto("FancyAutoPath"); //
                autoCommand = autoChooser.getSelected();
                /*
                try{
                    // Load the path you want to follow using its name in the GUI
                    PathPlannerPath path = PathPlannerPath.fromPathFile("AutoFancyCurve");

                    // Create a path following command using AutoBuilder. This will also trigger event markers.
                    autoCommand = AutoBuilder.followPath(path);
                } catch (Exception e) {
                    DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
                    autoCommand =  Commands.none();
                }
                */
                break;
            default:
                autoCommand = Commands.print("No autonomous command configured");
        }
        return autoCommand;

    }

}
