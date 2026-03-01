// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeActuationConsts;
import frc.robot.generated.TunerConstants;

/**
 * The Actuation subsystem controls the actuation motor that moves a mechanism
 * to a specific position.
 */
public class IntakeActuation extends SubsystemBase {
  private TalonFX actuationMotor;

  private MotionMagicVoltage motionMagicControl;
  private NeutralOut stopMode;
  // private PIDController paddingPID = new PIDController(1.5, 0, 0.0);

  /**
   * Creates a new Actuation.
   */
  public IntakeActuation(int motorId) {
    actuationMotor = new TalonFX(motorId, TunerConstants.kCANBus);
    initActuationMotor();

    motionMagicControl = new MotionMagicVoltage(0).withEnableFOC(true);
    // posPID.setTolerance(1.5);

    stopMode = new NeutralOut();
  }

  /**
   * Initialize the actuation motor
   */
  public void initActuationMotor() {

    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configs.Feedback.SensorToMechanismRatio = IntakeActuationConsts.GearRatio;

    // configs.MotorOutput.DutyCycleNeutralDeadband = 0.05;

    configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    configs.CurrentLimits.SupplyCurrentLimit = 40;

    configs.MotionMagic.MotionMagicCruiseVelocity = 30;
    configs.MotionMagic.MotionMagicAcceleration = 50;
    configs.MotionMagic.MotionMagicJerk = 75;

    /*
     * Voltage-based velocity requires a feed forward to account for the back-emf of
     * the motor
     */
    configs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    configs.Slot0.kP = 30; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI = 1.0; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD = 0.1; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12
                             // volts / Rotation per second
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = actuationMotor.getConfigurator().apply(configs);
      if (status.isOK())
        break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
  }

  /**
   * Run the actuation motor to a given position
   * 
   * @param position in encoder value
   * @return a command that will run the actuation motor
   */
  public Command setPositionCommand(double position) {
    return new Command() {
      @Override
      public void execute() {
        setPosition(position);
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }

  /**
   * Run the actuation motor to a given position
   * 
   * @param position in encoder value
   */
  public void setPosition(double position) {
    // System.out.println("Actuator Up");

    actuationMotor.setControl(motionMagicControl.withPosition(position));

    // resetPosition();
  }

  /**
   * Run the actuation motor at a given percent
   * 
   * @param speed 1 to -1
   */
  public Command runActuatorPercent(double speed) {
    return new Command() {
      @Override
      public void execute() {
        actuationMotor.set(speed);
      }

      @Override
      public void end(boolean interrupted) {
        actuationMotor.set(0);
      }
    };
  }

  /**
   * wait until the actuation motor is at a given position
   * 
   * @param setPosition in degrees
   * @return a command that will wait until the actuation motor is at a given
   *         position
   */
  public Command waitUntilAtPosition(double setPosition) {
    return new Command() {
      @Override
      public boolean isFinished() {
        double currentPosition = getAngle();
        return Math.abs(
            currentPosition * IntakeActuationConsts.GearRatio - setPosition * IntakeActuationConsts.GearRatio) <= 0.5;
      }
    };
  }

  public Command resetAngleToZeroCommand() {
    return new Command() {
      @Override
      public void execute() {
        setPosition(Constants.AngleControllerConsts.ANGLE_CONTROLLER_REST_POS);
      }

      @Override
      public boolean isFinished() {
        return actuationMotor.getSupplyCurrent()
            .getValueAsDouble() >= Constants.AngleControllerConsts.CURRENT_TOLERANCE;
      }
    };
  }

  /**
   * Get the actuation motor's current position
   * 
   * @return the actuation motor's current position in degrees
   */
  public double getAngle() {
    // return throughboreEncoder.getAbsolutePosition() / actuationTicksPerDegree -
    // actuationOffset;
    return actuationMotor.getPosition().getValueAsDouble() / IntakeActuationConsts.GearRatio;
    // return actuationMotor.getPosition().getValueAsDouble() /
    // actuationTicksPerDegree;
  }

  public void stopMotor() {
    actuationMotor.setControl(stopMode);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}