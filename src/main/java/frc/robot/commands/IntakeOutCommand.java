package frc.robot.commands;

import static frc.robot.Subsystems.m_intakeActuation;
import static frc.robot.Subsystems.m_intakeRollers;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class IntakeOutCommand extends ParallelCommandGroup {
    public IntakeOutCommand(double intakePosition, double intakeVel, double intakeAcc) {
        addCommands(
                m_intakeActuation.setPositionCommand(intakePosition),
                m_intakeRollers.feedCommand(intakeVel, intakeAcc));
    }
}