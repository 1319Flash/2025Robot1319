// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AlignToReefTagRelative;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.Arm.ClawSubsystem;
import frc.robot.subsystems.Arm.ShoulderSubsystem;
import frc.robot.subsystems.Arm.TelescopeSubsystem;
import frc.robot.subsystems.Climb.ClimberSubsystem;
import frc.robot.subsystems.Climb.HatchSubsystem;
import frc.robot.subsystems.Climb.HookSubsystem;
import frc.robot.subsystems.Climb.PeriscopeSubsystem;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double percentSlow = 0.6;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final RollerSubsystem rollerSubsystem = new RollerSubsystem();
    public final ClawSubsystem clawSubsystem = new ClawSubsystem();
    public final HatchSubsystem hatchSubsystem = new HatchSubsystem();
    private final TelescopeSubsystem telescopeSubsystem = new TelescopeSubsystem();
    private final ShoulderSubsystem shoulderSubsystem = new ShoulderSubsystem();
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    private final HookSubsystem hookSubsystem = new HookSubsystem();
    private final PeriscopeSubsystem periscopeSubsystem = new PeriscopeSubsystem();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        for (int port = 5800; port <= 5809; port++) {
          PortForwarder.add(port, "limelight.local", port);
      }

        NamedCommands.registerCommand("Dump", rollerSubsystem.runRoller(rollerSubsystem, () -> 0.2, () -> 0)
                                                   .withTimeout(.5)
                                                   .andThen(rollerSubsystem.stopRoller(rollerSubsystem)));

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
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed * percentSlow) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed * percentSlow) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate * percentSlow) // Drive counterclockwise with negative X (left)
            )
        );

        driverController.x().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        ));

        driverController.a().whileTrue(new AlignToReefTagRelative(drivetrain));

        driverController.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(1).withVelocityY(0))
        );
        driverController.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-1).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        driverController.start().onTrue(hookSubsystem.toggleHook(hookSubsystem)
        .andThen(new WaitCommand(1))
        .andThen(climberSubsystem.toggleClimber(climberSubsystem))
        .andThen(periscopeSubsystem.togglePeriscope(periscopeSubsystem)));

        driverController.rightBumper().whileTrue(new InstantCommand(()-> percentSlow = 0.2));
        driverController.rightBumper().whileFalse(new InstantCommand(() -> percentSlow = 0.6));

        drivetrain.registerTelemetry(logger::telemeterize);

        rollerSubsystem.setDefaultCommand(
            rollerSubsystem.runRoller(
                rollerSubsystem, 
                () -> 0, 
                () -> 0));

        operatorController.rightBumper()
            .whileTrue(rollerSubsystem.runRoller(rollerSubsystem, () -> 0.2 , () -> 0));

        operatorController.povUp().onTrue(rollerSubsystem.extendRollerSolenoid(rollerSubsystem));

        clawSubsystem.setDefaultCommand(
            clawSubsystem.runGripper(
            clawSubsystem, 
            () -> 0, 
            () -> 0.027));

        operatorController.leftTrigger().whileTrue(clawSubsystem.runGripper(clawSubsystem , () -> 0.0 , () -> 1.0));
        operatorController.rightTrigger().whileTrue(clawSubsystem.runGripper(clawSubsystem, () -> 1.0, () -> 0.0));

        telescopeSubsystem.setDefaultCommand(
            telescopeSubsystem.telescope(
                telescopeSubsystem,
            () -> (operatorController.getRightY()) * 0.3));

        operatorController.povDown().onTrue(shoulderSubsystem.retreactShoulderSolenoid(shoulderSubsystem).andThen(shoulderSubsystem.setShoulderPosition(0.0)).andThen(shoulderSubsystem.retreactShoulderSolenoid(shoulderSubsystem)));
        operatorController.a().onTrue(shoulderSubsystem.retreactShoulderSolenoid(shoulderSubsystem).andThen(shoulderSubsystem.setShoulderPosition(0.21704)).andThen(shoulderSubsystem.extendShoulderSolenoid(shoulderSubsystem)));
        operatorController.x().onTrue(shoulderSubsystem.retreactShoulderSolenoid(shoulderSubsystem).andThen(shoulderSubsystem.setShoulderPosition(0.7)).andThen(shoulderSubsystem.extendShoulderSolenoid(shoulderSubsystem)));
        operatorController.b().onTrue(shoulderSubsystem.retreactShoulderSolenoid(shoulderSubsystem).andThen(shoulderSubsystem.setShoulderPosition(.95)).andThen(shoulderSubsystem.extendShoulderSolenoid(shoulderSubsystem)));
        operatorController.y().onTrue(shoulderSubsystem.retreactShoulderSolenoid(shoulderSubsystem).andThen(shoulderSubsystem.setShoulderPosition(1.45)).andThen(shoulderSubsystem.extendShoulderSolenoid(shoulderSubsystem)));

        
        operatorController.axisLessThan(1,-0.1).whileTrue(
            shoulderSubsystem.retreactShoulderSolenoid(shoulderSubsystem).andThen(shoulderSubsystem.runShoulderWithSpeed(() -> -(operatorController.getLeftY() * 0.2))))
        .onFalse(shoulderSubsystem.runShoulderWithSpeed(() -> 0.025).andThen(shoulderSubsystem.extendShoulderSolenoid(shoulderSubsystem)));
        
        operatorController.axisGreaterThan(1,0.1).whileTrue(
            shoulderSubsystem.retreactShoulderSolenoid(shoulderSubsystem).andThen(shoulderSubsystem.runShoulderWithSpeed(() -> -(operatorController.getLeftY() * 0.2))))
            .onFalse(shoulderSubsystem.runShoulderWithSpeed(() -> 0.025).andThen(shoulderSubsystem.extendShoulderSolenoid(shoulderSubsystem)));
          
        operatorController.start().and(operatorController.back())
            .onTrue(hatchSubsystem.toggleHatch(hatchSubsystem));  
        
        operatorController.leftBumper().onTrue(rollerSubsystem.retractRollerSolenoid(rollerSubsystem));
    }
    
    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

}