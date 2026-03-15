package frc.robot.commands.autos;

import static frc.robot.Subsystems.m_intakeActuation;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class NeutralOutpostAuto {
    private final AutoFactory m_factory;

    public NeutralOutpostAuto(AutoFactory factory) {
        m_factory = factory;
    }

    public AutoRoutine neutralZoneAuto() {
        final AutoRoutine routine = m_factory.newRoutine("Neutral Zone Outpost");
        final AutoTrajectory startPickupAndIntake = routine.trajectory("OutpostNetural");

        routine.active().onTrue(
            startPickupAndIntake.cmd());
                // startPickupAndIntake.resetOdometry()
                //         .andThen(startPickupAndIntake.cmd()));

        return routine;
    }

}