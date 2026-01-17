// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

/**
 * The Indexer class represents a subsystem that controls the ballTunnel motor.
 * It provides methods to initialize, run, and stop the ballTunnel motor, as well as
 * check its speed and create commands to control it.
 */
public class BallTunnel extends SubsystemBase {
  private TalonFX ballTunnelMotor;

  VelocityVoltage velocityControl;
  NeutralOut stopMode;
  
  /**
   * Creates a new ballTunnel.
   */
  public BallTunnel(int motorId) {
    ballTunnelMotor = new TalonFX(motorId);

    initBallTunnelMotor();

    velocityControl = new VelocityVoltage(0);

    stopMode = new NeutralOut();
  }
   
  /**
   * Initialize the ballTunnel motor
   */
  public void initBallTunnelMotor() {
    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    configs.CurrentLimits.SupplyCurrentLimit = 30;

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.kP = 0.6; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI = 0.0; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD = 0.001; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
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
        runIndexer(velocity, acceleration);
      }

      @Override
      public void end(boolean interrupted) {
        stopIndexerMotor();
      }
    };
  }


  /**
   * Run the ballTunnel motor at a given velocity and acceleration
   * @param velocity in rotations per second
   * @param acceleration in rotations per second squared
   */
  public void runIndexer(double velocity, double acceleration) {
    ballTunnelMotor.setControl(velocityControl
                            .withVelocity(velocity)
                            .withAcceleration(acceleration)
                          );
  }


  /**
   * Run the ballTunnel motor at a given percentage speed
   * @param speed 1 to -1
   * @return a command that will run the ballTunnel motor
   */
  public Command runIndexerPercent(double speed) {
    return new Command() {
      @Override
      public void initialize() {
        addRequirements(BallTunnel.this);
      }

      @Override
      public void execute() {
        ballTunnelMotor.set(speed);
      }

      @Override
      public void end(boolean interrupted) {
        ballTunnelMotor.set(0);
      }
    };
  }

  /**
   * Stop the ballTunnel motor
   */
  public void stopIndexerMotor() {
    ballTunnelMotor.setControl(stopMode);
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
      }
    };
  }

  /**
   * Get the current speed of the ballTunnel
   * @return the speed of the ballTunnel in rotations per second
   */
  public double getVelocity() {
    return ballTunnelMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println("ball tunnel: " + ballTunnelMotor.getVelocity().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}