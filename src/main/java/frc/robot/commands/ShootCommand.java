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
            m_AngleController.setPositionCommand(shooterAngle),
            // m_ballTunnel.runBallTunnelCommand(shooterSpeed, 1)
            new SequentialCommandGroup(
                m_AngleController.waitUntilAtPosition(shooterAngle),
                m_shooter.runShooterCommand(shooterSpeed, 100)));
    }
}
