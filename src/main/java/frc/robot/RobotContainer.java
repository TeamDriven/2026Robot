// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Subsystems.m_angleController;
import static frc.robot.Subsystems.m_ballTunnel;
import static frc.robot.Subsystems.m_intakeActuation;
import static frc.robot.Subsystems.m_intakeRollers;
import static frc.robot.Subsystems.m_shooter;

import java.time.Instant;
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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.*;

import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.BallTunnel;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.Constants.DrivetrainConst;
import frc.robot.commands.IntakeOutCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.autos.MiddleAuto;
import frc.robot.commands.autos.OutpostAuto;
import frc.robot.commands.autos.OutpostNeutralAuto;

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
        // private final OutpostAuto outpostAuto;
        //private final NeutralDepotAuto neutralZoneDepotAuto;
        //private final DepotAuto depotAuto;
        private final OutpostNeutralAuto outpostNeutralAuto;
        private final MiddleAuto middleAuto;
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

        public static double xPose;

        public static double yPose;

        public static final PowerDistribution m_pdh = new PowerDistribution(30, ModuleType.kRev);

        public RobotContainer() {
                SmartDashboard.putData("Field", m_field);
                autoFactory = drivetrain.createAutoFactory();
                //outpostAuto = new OutpostAuto(autoFactory);
                //neutralZoneDepotAuto = new NeutralDepotAuto(autoFactory);
                //depotAuto = new DepotAuto(autoFactory);
                outpostNeutralAuto = new OutpostNeutralAuto(autoFactory);
                middleAuto = new MiddleAuto(autoFactory);
                //autoChooser.addRoutine("Depot Auto", depotAuto::simplePathAuto);
                //autoChooser.addRoutine("Outpost Auto", outpostAuto::simplePathAuto);
                //autoChooser.addRoutine("Neutral Zone Depot Auto", neutralZoneDepotAuto::neutralZoneAuto);
                autoChooser.addRoutine("OutpostNetural", outpostNeutralAuto::simplePathAuto);
                autoChooser.addRoutine("Middle Auto", middleAuto::middleAuto);
                SmartDashboard.putData("Auto Chooser", autoChooser);

                frontLeft = drivetrain.getState().ModulePositions[0];
                frontRight = drivetrain.getState().ModulePositions[1];
                backLeft = drivetrain.getState().ModulePositions[2];
                backRight = drivetrain.getState().ModulePositions[3];
                m_intakeActuation.setPosition(m_intakeActuation.getCurrentPosition());
                configureBindings();

        }

        public void robotPeriodic(){
                xPose = drivetrain.getPose().getX();
                yPose = drivetrain.getPose().getY();
                SmartDashboard.putNumber("XPose", xPose);
                SmartDashboard.putNumber("YPose", yPose);
                
              
        }

        private void configureBindings() {
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(Controls.driveRequest()));

                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));


                drivetrain.registerTelemetry(logger::telemeterize);

                // joystick.povDown().onTrue(drivetrain.applyRequest(Controls.localHeading(Constants.FieldConst.BLUE_HUB))).onFalse(drivetrain.applyRequest(Controls.driveRequest()));
                
              joystick.povDown().onTrue(drivetrain.applyRequest(Controls.localHeading(Robot.alliance == Alliance.Red ? Constants.FieldConst.RED_HUB : Constants.FieldConst.BLUE_HUB))).onFalse(drivetrain.applyRequest(Controls.driveRequest()));
                //joystick.povUp().onTrue(m_intakeActuation.intakeInSlowCommand());
                

                // Intake
                joystick.b().onTrue(m_intakeActuation.setPositionUntilSupply(1.39));
                joystick.x().onTrue(m_intakeActuation.setPositionUntilSupply(0).andThen(new WaitCommand(5)).andThen(m_intakeRollers.stopIntakeCommand()));
                joystick.leftBumper().whileTrue(m_intakeRollers.feedCommand(90, 100)).whileFalse(m_intakeRollers.stopIntakeCommand());

                // joystick.rightBumper().toggleOnTrue(new ShootCommand(29, 10, 62.5))
                //                 .toggleOnFalse(m_angleController.runOnce(() -> m_angleController.setPosition(2)).alongWith(
                //                                 new InstantCommand(() -> m_angleController.setPosition(0)),
                //                                 new InstantCommand(() -> m_shooter.stopMotors()),
                //                                 new InstantCommand(() -> m_ballTunnel.stopBallTunnel())));


        //        joystick.rightBumper().toggleOnTrue(new ShootCommand(() -> m_shooter.calcSpeed(), 11, 62.5))
        //                 .toggleOnFalse(m_angleController.runOnce(() -> m_angleController.setPosition(2)).alongWith(
        //                                 new InstantCommand(() -> m_angleController.setPosition(0)),
        //                                 new InstantCommand(() -> m_shooter.stopMotors()),
        //                                 new InstantCommand(() -> m_ballTunnel.stopBallTunnel())));

               joystick.rightBumper().toggleOnTrue(new ShootCommand(()->0, 11, 62.5))
                        .toggleOnFalse(m_angleController.runOnce(() -> m_angleController.setPosition(2)).alongWith(
                                        new InstantCommand(() -> m_angleController.setPosition(0)),
                                        new InstantCommand(() -> m_shooter.stopMotors()),
                                        new InstantCommand(() -> m_ballTunnel.stopBallTunnel()),
                                        m_shooter.stopShooterCommand()));


                                        //m_intakeActuation.setPositionUntilSupply(1.39)));


                // joystick.rightTrigger().toggleOnTrue(new ShootCommand(75,30, 62.5))
                //         .toggleOnFalse(m_angleController.runOnce(() -> m_angleController.setPosition(2)).alongWith(
                //                         new InstantCommand(() -> m_angleController.setPosition(0)),
                //                         new InstantCommand(() -> m_shooter.stopMotors()),
                //                         new InstantCommand(() -> m_ballTunnel.stopBallTunnel())));
                                        
                joystick.leftTrigger().whileTrue(m_ballTunnel.spitCommand(20, 62, 14, 100)
                .alongWith(m_angleController.runOnce(() -> m_angleController.setPosition(2)))
                .alongWith(m_intakeRollers.feedCommand(85, 100))
                .alongWith(m_intakeActuation.intakeInSlowCommand(0.5)))
                .whileFalse(new InstantCommand(() -> m_ballTunnel.stopBallTunnel()));

                joystick.a().onTrue(new InstantCommand(() -> m_ballTunnel.runBallTunnel(-62.5, 100)).alongWith(new InstantCommand(() -> m_ballTunnel.runHopper(-62.5, 100)))).onFalse(new InstantCommand(() -> m_ballTunnel.stopBallTunnel()));
                joystick.y().onTrue(new InstantCommand(() -> m_ballTunnel.runBallTunnel(62.5, 100)).alongWith(new InstantCommand(() -> m_ballTunnel.runHopper(62.5, 100)))).onFalse(new InstantCommand(() -> m_ballTunnel.stopBallTunnel()));

                // Reset Heading
                joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()).alongWith(new InstantCommand(() -> m_angleController.setPosition(0))));
        }

        public Command getAutonomousCommand() {
                return autoChooser.selectedCommand();
        }
}