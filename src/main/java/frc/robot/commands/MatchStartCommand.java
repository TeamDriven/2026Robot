package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import static frc.robot.Subsystems.m_actuation;

import frc.robot.Constants.ActuationConsts;

public class MatchStartCommand extends SequentialCommandGroup{
    public MatchStartCommand() {
        addCommands(
            m_actuation.setPositionCommand(ActuationConsts.DOWN_POS)
        );
    }
}
