// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Subsystems.m_AngleController;
import static frc.robot.Subsystems.m_ballTunnel;
import static frc.robot.Subsystems.m_intakeActuation;
import static frc.robot.Subsystems.m_intakeRollers;
import static frc.robot.Subsystems.m_shooter;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.Constants.DrivetrainConst;
import frc.robot.commands.IntakeOutCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.autos.DepotAuto;
import frc.robot.commands.autos.NeutralDepotAuto;
import frc.robot.commands.autos.NeutralOutpostAuto;
import frc.robot.commands.autos.OutpostAuto;
import static frc.robot.Controls.joystick;

public class RobotContainer {

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(DrivetrainConst.MaxSpeed * 0.1)
                        .withRotationalDeadband(DrivetrainConst.MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final Telemetry logger = new Telemetry(DrivetrainConst.MaxSpeed);

        public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        /* Path follower */
        public static AutoFactory autoFactory;
        private final OutpostAuto outpostAuto;
        private final NeutralOutpostAuto neutralZoneOutpostAuto;
        private final NeutralDepotAuto neutralZoneDepotAuto;
        private final DepotAuto depotAuto;
        private final AutoChooser autoChooser = new AutoChooser();

        public static SwerveModulePosition frontRight;

        public static SwerveModulePosition backRight;

        public static SwerveModulePosition frontLeft;

        public static SwerveModulePosition backLeft;

        public static final Translation2d m_frontLeftLocation = new Translation2d(TunerConstants.kFrontLeftXPos,
                        TunerConstants.kFrontLeftYPos);

        public static final Translation2d m_frontRightLocation = new Translation2d(TunerConstants.kFrontRightXPos,
                        TunerConstants.kFrontRightYPos);

        public static final Translation2d m_backLeftLocation = new Translation2d(TunerConstants.kBackLeftXPos,
                        TunerConstants.kBackLeftYPos);

        public static final Translation2d m_backRightLocation = new Translation2d(TunerConstants.kBackRightXPos,
                        TunerConstants.kBackRightYPos);

        public static final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation,
                        m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

        public static Field2d m_field = new Field2d();

        public static final PowerDistribution m_pdh = new PowerDistribution(30, ModuleType.kRev);

        public static boolean isShooting = false;

        public RobotContainer() {
                SmartDashboard.putData("Field", m_field);
                autoFactory = drivetrain.createAutoFactory();
                outpostAuto = new OutpostAuto(autoFactory);
                neutralZoneOutpostAuto = new NeutralOutpostAuto(autoFactory);
                neutralZoneDepotAuto = new NeutralDepotAuto(autoFactory);
                depotAuto = new DepotAuto(autoFactory);

                autoChooser.addRoutine("Depot Auto", depotAuto::simplePathAuto);
                autoChooser.addRoutine("Outpost Auto", outpostAuto::simplePathAuto);
                autoChooser.addRoutine("Neutral Zone Depot Auto", neutralZoneDepotAuto::neutralZoneAuto);
                autoChooser.addRoutine("Neutral Zone Outpost Auto", neutralZoneOutpostAuto::neutralZoneAuto);
                SmartDashboard.putData("Auto Chooser", autoChooser);

                frontLeft = drivetrain.getState().ModulePositions[0];
                frontRight = drivetrain.getState().ModulePositions[1];
                backLeft = drivetrain.getState().ModulePositions[2];
                backRight = drivetrain.getState().ModulePositions[3];

                configureBindings();
        }

        private void configureBindings() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(Controls.driveRequest()));

                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.
                // Controlls.joystick.back().and(Controlls.joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                // Controlls.joystick.back().and(Controlls.joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                // Controlls.joystick.start().and(Controlls.joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                // Controlls.joystick.start().and(Controlls.joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                drivetrain.registerTelemetry(logger::telemeterize);

                // Controls.autoLineUpOn.onTrue(drivetrain.applyRequest(Controls.localHeading(Constants.FieldConst.RED_HUB)));
                // Controls.autoLineUpOff.onTrue(drivetrain.applyRequest(Controls.driveRequest()));
                // Controls.autoLineUpOn
                // .onTrue(drivetrain.applyRequest(Controls.goToPositionAndRotation(
                // new Pose2d(14, 5, new Rotation2d(0)), Constants.FieldConst.RED_HUB)))
                // .onFalse(drivetrain.applyRequest(Controls.driveRequest()));

                // Intake
                joystick.leftBumper().onTrue(new IntakeOutCommand(1.43, 70, 100)).onFalse(m_intakeRollers.stopIntakeCommand());

                // Shoot
                joystick.rightBumper().and(() -> !isShooting).onTrue(new ShootCommand(20, 10, 14).alongWith(new InstantCommand(() -> changeIsShooting(true))));
                joystick.rightBumper().and(() -> isShooting).onTrue(new ShootCommand(0, 0.5, 0).alongWith(new InstantCommand(() -> changeIsShooting(false))));

                // Spit
                joystick.b().and(() -> !isShooting).onTrue(m_ballTunnel.spitCommand(14, 100).alongWith(new InstantCommand(() -> changeIsShooting(true))));
                joystick.b().and(() -> !isShooting).onTrue(new InstantCommand(() -> m_ballTunnel.stopBallTunnel()).alongWith(new InstantCommand(() -> changeIsShooting(true))));

                // BallTunnel Manually
                joystick.pov(0).onTrue(m_ballTunnel.runBallTunnelCommand(14, 100)).onFalse(new InstantCommand(() -> m_ballTunnel.stopBallTunnel()));
                joystick.pov(180).onTrue(m_ballTunnel.runBallTunnelCommand(14, 100)).onFalse(new InstantCommand(() -> m_ballTunnel.stopBallTunnel()));

                // Small Mode
                joystick.back().onTrue(m_AngleController.setPositionCommand(0.5).alongWith(m_intakeActuation.setPositionCommand(0)));

                // Reset Heading
                joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
                joystick
                                .button(8)
                                .onTrue(
                                                Commands.runOnce(
                                                                () -> drivetrain.setPose(
                                                                                new Pose2d(drivetrain.getPose()
                                                                                                .getTranslation(),
                                                                                                new Rotation2d())),
                                                                drivetrain)
                                                                .ignoringDisable(true));
        }

        public Command getAutonomousCommand() {
                return autoChooser.selectedCommand();
        }

        public void changeIsShooting(boolean input) {
                isShooting = input;
        }
}
