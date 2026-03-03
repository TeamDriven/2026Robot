package frc.robot.commands;

import static frc.robot.Subsystems.m_angleController;
import static frc.robot.Subsystems.m_ballTunnel;
import static frc.robot.Subsystems.m_shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ShootCommand extends ParallelCommandGroup {
    public ShootCommand(double shooterSpeed, double shooterAngle, double ballTunnelSpeed) {
        addCommands(
                m_angleController.runOnce(() -> m_angleController.setPosition(shooterAngle)),
                m_shooter.runOnce(() -> m_shooter.runShooter(shooterSpeed, 100)),
                new SequentialCommandGroup(
                        m_angleController.waitUntilAtPosition(shooterAngle),
                        // m_ballTunnel.waitUntilAtSpeed(ballTunnelSpeed),
                        // m_shooter.waitUntilAtSpeed(shooterSpeed),
                        new ParallelCommandGroup(
                                // you will not want to run the ball tunnel until you know that shooter is at
                                // the correct speed
                                m_ballTunnel.runBallTunnelCommand(ballTunnelSpeed, 20))));
    }
}
