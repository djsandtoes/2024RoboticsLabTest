// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.MotorSpeedConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;

public class ExampleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private WPI_TalonSRX talonSRX1;
  private final Encoder throughBoreEncoder;

  public final PIDController pid;
  // Fast, not smooth:   kP = 0.0000240; kI = 0.0000000; kD = 0.0000000;
  // Tuned smoother:     kP = 0.0000050; kI = 0.0000008; kD = 0.0000001;
  private final double kP = 0.0000240;
  private final double kI = 0.0000000; 
  private final double kD = 0.0000000;
  double setpoint = 0;
  double pid_val;
  boolean pid_enable = false;

  public ExampleSubsystem() {
    talonSRX1 = new WPI_TalonSRX(8);
    //talonSRX1.set(0.2);

    throughBoreEncoder = new Encoder(0, 1, true, Encoder.EncodingType.k1X);
    throughBoreEncoder.reset();
    // Configures the encoder to return a distance of 4 for every 256 pulses
    // Also changes the units of getRate
    //throughBoreEncoder.setDistancePerPulse(4.0/256.0);
    // Configures an encoder to average its period measurement over 5 samples
    // Can be between 1 and 127 samples
    //throughBoreEncoder.setSamplesToAverage(5);

    pid = new PIDController(kP, kI, kD);
    pid.reset();
 
    SmartDashboard.putNumber("Encoder Distance Setpoint", setpoint);
    SmartDashboard.putNumber("kP", kP);
    SmartDashboard.putNumber("kI", kI);
    SmartDashboard.putNumber("kD", kD);
    SmartDashboard.putBoolean("PID Enable", pid_enable);

    }

  public void SetSpeed (double speed) {
    talonSRX1.set(speed);
  }
  public double GetSpeed () {
    return talonSRX1.get();
  }
  public double GetDistance () {
    return throughBoreEncoder.get();
  } 

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {

        });
  }

  public Command crawlForward() {
    return runOnce(() -> SetSpeed(MotorSpeedConstants.CRAWL_FORWARD));
  }
  public Command walkForward() {
    return runOnce(() -> SetSpeed(MotorSpeedConstants.WALK_FORWARD));
  }
  public Command runForward() {
    return runOnce(() -> SetSpeed(MotorSpeedConstants.RUN_FORWARD));
  }
  public Command crawlBackward() {
    return runOnce(() -> SetSpeed(MotorSpeedConstants.CRAWL_BACKWARD));
  }
  public Command walkBackward() {
    return runOnce(() -> SetSpeed(MotorSpeedConstants.WALK_BACKWARD));
  }
  public Command runBackward() {
    return runOnce(() -> SetSpeed(MotorSpeedConstants.RUN_BACKWARD));
  }
  public Command Halt() {
    return runOnce(() -> SetSpeed(0));
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //talonSRX1.set(0.1);
    SmartDashboard.putNumber("Motor Speed", talonSRX1.get());
    SmartDashboard.putNumber("Motor (Encoder) Rate", throughBoreEncoder.getRate());
    SmartDashboard.putNumber("Motor (Encoder) Distance", throughBoreEncoder.getDistance());

    pid.setP(SmartDashboard.getNumber("kP", 0));
    pid.setI(SmartDashboard.getNumber("kI", 0));
    pid.setD(SmartDashboard.getNumber("kD",0));

    setpoint=SmartDashboard.getNumber("Encoder Distance Setpoint", 0);
    pid_enable=SmartDashboard.getBoolean("PID Enable", false);
    if (pid_enable) {
      talonSRX1.set(MathUtil.clamp(pid.calculate(throughBoreEncoder.getDistance(), setpoint), 
                                   MotorSpeedConstants.RUN_BACKWARD, 
                                   MotorSpeedConstants.RUN_FORWARD));
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
