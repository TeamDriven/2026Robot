// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Subsystems.m_ballTunnel;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.BallTunnelConsts;
import frc.robot.generated.TunerConstants;

/**
 * The Indexer class represents a subsystem that controls the ballTunnel motor.
 * It provides methods to initialize, run, and stop the ballTunnel motor, as well as
 * check its speed and create commands to control it.
 */
public class BallTunnel extends SubsystemBase {
  private TalonFX ballTunnelMotor;
  private TalonFX diverterMotor;

  VelocityVoltage velocityControl;
  NeutralOut stopMode;
  
  /**
   * Creates a new ballTunnel.
   */
  public BallTunnel(int DMotorId, int BMotorId) {
    ballTunnelMotor = new TalonFX(BMotorId, TunerConstants.kCANBus);
    diverterMotor = new TalonFX(DMotorId, TunerConstants.kCANBus);

    initMotors();

    velocityControl = new VelocityVoltage(0).withEnableFOC(true);

    stopMode = new NeutralOut();
  }
   
  /**
   * Initialize the ballTunnel motor
   */
  public void initMotors() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    
    configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configs.Feedback.SensorToMechanismRatio = BallTunnelConsts.MainGearRatio;
    
    configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    configs.CurrentLimits.SupplyCurrentLimit = 30;

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.kP = 0;// 0.0000001; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI = 0.0; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD = 0.1; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV = 0.10; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 12;
    configs.Voltage.PeakReverseVoltage = -12;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = ballTunnelMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    configs = new TalonFXConfiguration();

    configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    configs.Feedback.SensorToMechanismRatio = BallTunnelConsts.DivergentGearRatio;

    configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    configs.CurrentLimits.SupplyCurrentLimit = 30;

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.kP = 0.1; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI = 0.0; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD = 0.001; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV = 0.10; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 12;
    configs.Voltage.PeakReverseVoltage = -12;

    status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = diverterMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
  }

  /**
   * Run the ballTunnel motor at a given velocity and acceleration
   * @param velocity in rotations per second
   * @param acceleration in rotations per second squared
   * @return a command that will run the ballTunnel motor
   */
  public Command runBallTunnelCommand(double velocity, double acceleration){
    return new Command() {
      @Override
      public void initialize() {
        addRequirements(BallTunnel.this);
      }

      @Override
      public void execute() {
        runBallTunnel(velocity, acceleration);
      }

      @Override
      public void end(boolean interrupted) {
        stopBallTunnelMotor();
      }
    };
  }


  /**
   * Run the ballTunnel motor at a given velocity and acceleration
   * @param velocity in rotations per second
   * @param acceleration in rotations per second squared
   */
  public void runBallTunnel(double velocity, double acceleration) {
    ballTunnelMotor.setControl(velocityControl
                            .withVelocity(velocity)
                            .withAcceleration(acceleration)
                          );
    diverterMotor.setControl(velocityControl
                            .withVelocity(velocity)
                            .withAcceleration(acceleration)
                          );
  }

  /**
   * Run the ballTunnel motor at a given velocity and acceleration
   * @param velocity in rotations per second
   * @param acceleration in rotations per second squared
   * @return a command that will run the ballTunnel motor
   */
  public Command spitCommand(double velocity, double acceleration){
    return new Command() {
      @Override
      public void initialize() {
        addRequirements(BallTunnel.this);
      }

      @Override
      public void execute() {
        spit(velocity, acceleration);
      }

      @Override
      public void end(boolean interrupted) {
        stopBallTunnelMotor();
      }
    };
  }

  /**
   * Run the ballTunnel motor at a given velocity and acceleration
   * @param velocity in rotations per second
   * @param acceleration in rotations per second squared
   */
  public void spit(double velocity, double acceleration) {
    ballTunnelMotor.setControl(velocityControl
                            .withVelocity(velocity)
                            .withAcceleration(acceleration)
                          );
    diverterMotor.setControl(velocityControl
                            .withVelocity(-10 * velocity) // -
                            .withAcceleration(acceleration)
                          );
  }


  /**
   * Run the ballTunnel motor at a given percentage speed
   * @param speed 1 to -1
   * @return a command that will run the ballTunnel motor
   */
  public Command runBallTunnelPercent(double speed) {
    return new Command() {
      @Override
      public void initialize() {
        addRequirements(BallTunnel.this);
      }

      @Override
      public void execute() {
        // ballTunnelMotor.set(speed);
        diverterMotor.set(speed);
      }

      @Override
      public void end(boolean interrupted) {
        // ballTunnelMotor.set(0);
        diverterMotor.set(0);
      }
    };
  }

  /**
   * Stop the ballTunnel motor
   */
  public void stopBallTunnelMotor() {
    ballTunnelMotor.setControl(stopMode);
    diverterMotor.setControl(stopMode);
  }

  /**
   * Checks if the ballTunnel is at a certain speed
   * @param velocity the speed to check for in rotations per second
   * @return a command that will wait until the ballTunnel is at a certain speed
   */
  public Command checkIfAtSpeedSupplier(DoubleSupplier velocity) {
    return new Command() {
      @Override
      public void initialize() {
      }

      @Override
      public void execute() {
      }

      @Override
      public void end(boolean interrupted) {
      }

      @Override
      public boolean isFinished() {
        if (((Double) velocity.getAsDouble()).equals(Double.NaN)) {
          return true;
        }
        return ballTunnelMotor.getVelocity().getValueAsDouble() >= velocity.getAsDouble() * 0.90;
        // return true;
      }
    };
  }

  
  public Command waitUntilAtSpeed(double speed) {
    return new WaitUntilCommand(() -> {
      return Math.abs(diverterMotor.getVelocity().getValueAsDouble() - speed) <= 2;
    });
  }

  /**
   * Get the current speed of the ballTunnel
   * @return the speed of the ballTunnel in rotations per second
   */
  public double getVelocity() {
    return diverterMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println("ball tunnel: " + ballTunnelMotor.getVelocity().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}