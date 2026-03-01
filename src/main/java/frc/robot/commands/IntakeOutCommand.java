package frc.robot.commands;

import static frc.robot.Subsystems.m_AngleController;
import static frc.robot.Subsystems.m_ballTunnel;
import static frc.robot.Subsystems.m_intakeActuation;
import static frc.robot.Subsystems.m_intakeRollers;
import static frc.robot.Subsystems.m_shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class IntakeOutCommand extends ParallelCommandGroup {
    public IntakeOutCommand(double intakePosition, double intakeVel, double intakeAcc) {
        addCommands(
                m_intakeActuation.runOnce(() -> m_intakeActuation.setPosition(intakePosition)),
                m_intakeRollers.runOnce(() -> m_intakeRollers.feedMotor(intakeVel, intakeAcc)));
    }
}