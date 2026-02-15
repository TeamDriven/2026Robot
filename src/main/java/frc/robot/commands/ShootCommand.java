package frc.robot.commands;

import static frc.robot.Subsystems.m_AngleController;
import static frc.robot.Subsystems.m_ballTunnel;
import static frc.robot.Subsystems.m_shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ShootCommand extends ParallelCommandGroup {
    public ShootCommand(double shooterSpeed, double shooterAngle, double ballTunnelSpeed) {
        addCommands(
            m_shooter.runShooterCommand(shooterSpeed, 100),
            m_AngleController.setPositionCommand(shooterAngle),
            new SequentialCommandGroup(
                new WaitUntilCommand(() -> Math.abs(m_shooter.getVelocity() - shooterSpeed) <= 1), // 1 is a tolerence
                m_AngleController.waitUntilAtPosition(shooterAngle),
                m_ballTunnel.runBallTunnelCommand(shooterAngle, ballTunnelSpeed))
        );
    }
}
