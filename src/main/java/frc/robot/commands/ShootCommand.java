package frc.robot.commands;

import static frc.robot.RobotContainer.drivetrain;
import static frc.robot.Subsystems.m_angleController;
import static frc.robot.Subsystems.m_ballTunnel;
import static frc.robot.Subsystems.m_intakeActuation;
import static frc.robot.Subsystems.m_intakeRollers;
import static frc.robot.Subsystems.m_shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeActuation;
import static frc.robot.RobotContainer.xPose;
import static frc.robot.RobotContainer.yPose;

public class ShootCommand extends ParallelCommandGroup {
    public ShootCommand(DoubleSupplier shooterSpeed, double shooterAngle, double ballTunnelSpeed) {

        addCommands(
                m_angleController.runOnce(() -> m_angleController.setPosition(shooterAngle)),
                m_shooter.runShooterCommand(100),
                new SequentialCommandGroup(
                        m_angleController.waitUntilAtPosition(shooterAngle),
                        new ParallelCommandGroup(
                                // you will not want to run the ball tunnel until you know that shooter is at
                                // the correct speed
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> m_ballTunnel.runBallTunnel(ballTunnelSpeed, 30)),
                                        new InstantCommand(() -> m_ballTunnel.runHopper(-ballTunnelSpeed, 20)),
                                        new WaitCommand(0.3),
                                        new InstantCommand(() -> m_ballTunnel.runHopper(ballTunnelSpeed, 20))),
                                Commands.waitSeconds(2),
                                m_intakeRollers.feedCommand(85, 100),
                                m_intakeActuation.intakeInSlowCommand(0.35))));
                                

    }
}
