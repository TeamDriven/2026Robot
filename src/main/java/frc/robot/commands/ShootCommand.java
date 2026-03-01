package frc.robot.commands;

import static frc.robot.Subsystems.m_AngleController;
import static frc.robot.Subsystems.m_ballTunnel;
import static frc.robot.Subsystems.m_shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ShootCommand extends ParallelCommandGroup {
    public ShootCommand(double shooterSpeed, double shooterAngle, double ballTunnelSpeed) {
        addCommands(
                m_AngleController.runOnce(() -> m_AngleController.setPosition(shooterAngle)),
                m_shooter.runOnce(() -> m_shooter.runShooter(shooterSpeed, 100)),
                new SequentialCommandGroup(
                    m_AngleController.waitUntilAtPosition(shooterAngle),
                    m_ballTunnel.runOnce(() -> m_ballTunnel.runBallTunnel(ballTunnelSpeed, 35))));
    }
}
