package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import frc.robot.Constants;
import frc.robot.Constants.AngleControllerConsts;

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

/**
 * The AngleController class represents a subsystem that controls the angle of the shooter.
 */
public class AngleController extends SubsystemBase{
  // private PowerDistribution pdp = new PowerDistribution(30, ModuleType.kRev);
  private TalonFX angleMotor;

  MotionMagicVoltage motionMagicControl;
  NeutralOut stopMode;

  public AngleController(int motorId) {
    angleMotor = new TalonFX(motorId);
    initAngleMotor();

    motionMagicControl = new MotionMagicVoltage(0);
              
    stopMode = new NeutralOut();
  }

  /**
   * Initialize the Angle Controller motor
   */
  public void initAngleMotor() {

    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    // configs.MotorOutput.DutyCycleNeutralDeadband = 0.001;

    configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    configs.CurrentLimits.SupplyCurrentLimit = 40;

    configs.MotionMagic.MotionMagicCruiseVelocity = 15;
    configs.MotionMagic.MotionMagicAcceleration = 20;
    configs.MotionMagic.MotionMagicJerk = 50;

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    configs.Slot0.kP = 60; //25 // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI = 0.5; //4 // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD = 0.0; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second

    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = angleMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
  }

  /**
   * Run the Angle Controller motor to a given position
   * @param position in degrees
   * @return a command that will run the Angle Controller motor
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
   * Run the Angle Controller motor to a given position
   * @param position in degrees
   * @return a command that will run the Angle Controller motor
   */
  public Command setPositionCommandSupplier(DoubleSupplier position) {
    return new Command() {
      @Override
      public void execute() {
        if (!((Double) position.getAsDouble()).equals(Double.NaN)) {
          setPosition(position.getAsDouble());
        }
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }

  /**
   * Run the Angle Controller motor to a given position
   * @param position in degrees
   */
  public void setPosition(double position) {
    angleMotor.setControl(motionMagicControl
                              .withPosition(position  * AngleControllerConsts.GEAR_RATIO)
                            );
  }

  /**
   * Run the Angle Controller motor at a given percent
   * @param percent 1 to -1
   */
  public Command anglePercentControl(double power) {
    return new Command() {
      @Override
      public void execute() {
        angleMotor.set(power);
      }

      @Override
      public void end(boolean interrupted) {
        angleMotor.set(0);
      }
    };
  }

  /**
   * wait until the Angle Controller motor is at a given position
   * @param setPosition in degrees
   * @return a command that will wait until the Angle Controller motor is at a given position
   */
  public Command waitUntilAtPosition(double setPosition) {
    // return new WaitUntilCommand(() -> Math.abs(angleMotor.getPosition().getValueAsDouble() - setPosition  * angleTicksPerDegree) <= 0.5);
    return new Command() {
      @Override
      public boolean isFinished() {
        double currentPosition = angleMotor.getPosition().getValueAsDouble();
        return Math.abs(currentPosition - setPosition  * AngleControllerConsts.GEAR_RATIO) <= 0.5;
      }
    };
  }

  /**
   * wait until the Angle Controller motor is at a given position
   * @param setPosition in degrees
   * @return a command that will wait until the Angle Controller motor is at a given position
   */
  public Command waitUntilAtPositionSupplier(DoubleSupplier setPosition) {
    return new Command() {
      @Override
      public boolean isFinished() {
        if (((Double) setPosition.getAsDouble()).equals(Double.NaN)) {
          return true;
        }
        double currentPosition = angleMotor.getPosition().getValueAsDouble();
        return Math.abs(currentPosition - setPosition.getAsDouble() * AngleControllerConsts.GEAR_RATIO) <= 0.1;
      }
    };
  }
  
  /**
   * Stops the Angle Controller motor's movement
   */
  public void stopMotor() {
    angleMotor.setControl(stopMode);
  }

  /**
   * Get the current angle of the Angle Controller motor
   * @return the current angle in degrees
   */
  public double getAngle() {
    return angleMotor.getPosition().getValueAsDouble() / AngleControllerConsts.GEAR_RATIO;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}