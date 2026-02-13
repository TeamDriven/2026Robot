package frc.robot.commands.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Controls;
import frc.robot.RobotContainer;

public class NeutralZoneAuto {
    private final AutoFactory m_factory;

    public NeutralZoneAuto(AutoFactory factory) {
        m_factory = factory;
    }

    public AutoRoutine neutralZoneAuto() {
        final AutoRoutine routine = m_factory.newRoutine("Neutral Zone");
        final AutoTrajectory startPickupAndIntake = routine.trajectory("NeutralZone", 0);
        final AutoTrajectory startshooter = routine.trajectory("NeutralZone", 1);
        final AutoTrajectory beginShooting = routine.trajectory("NeutralZone", 2);
        final AutoTrajectory climb1 = routine.trajectory("NeutralZone", 3);

        routine.active().onTrue(
                startPickupAndIntake.resetOdometry()
                        .andThen(startPickupAndIntake.cmd()));

        startPickupAndIntake.done().onTrue(Commands.sequence(
                new WaitCommand(1),
                // start intake code
                startshooter.cmd()));

        startshooter.done().onTrue(Commands.sequence(
                new WaitCommand(1),
                // start shooter
                beginShooting.cmd()));
                
                beginShooting.done().onTrue(Commands.sequence(
                new WaitCommand(1),
                // start climbing code
                climb1.cmd()));
        

        return routine;
    }


}