// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Hardware;
import frc.robot.Robot;

public class Elevator extends SubsystemBase {

  private double errorSum, lastError, lastTime;

  /** Creates a new Elevator. */
  public Elevator() {
  
    Hardware.gearbox = DCMotor.getVex775Pro(2);
    Hardware.simulation = new ElevatorSim(Hardware.gearbox, Constants.GEAR_REDUCTION, Constants.CARRIAGE_MASS,
      Constants.PULLEY_RADIUS, Constants.MIN_HEIGHT, Constants.MAX_HEIGHT);

    Hardware.left = new WPI_TalonSRX(0);
    Hardware.right = new WPI_TalonSRX(1);

    Hardware.left.configFactoryDefault();
    Hardware.right.configFactoryDefault();

    Hardware.left.setInverted(true);
    Hardware.right.setInverted(false);

    Hardware.left.follow(Hardware.right);
    Hardware.right.setSelectedSensorPosition(0);
    
    Hardware.encoder = new Encoder(0, 1);
    Hardware.encoder.reset();
    Hardware.encoder.setDistancePerPulse(Constants.Distance_PER_PULSE);
    Hardware.encoderSim = new EncoderSim(Hardware.encoder);

    Hardware.limitSwitch = new DigitalInput(2);

    errorSum = lastError = lastTime = 0.0;
  }

  public void move(Constants.MOVE move)
  {
    double pidOutput = calc(move);
    
    Hardware.right.set(pidOutput);
    
    if(!Robot.isRobotConnected())
    {
      Hardware.simulation.setInput(Hardware.right.getMotorOutputVoltage());
      Hardware.simulation.update(0.020);
      Hardware.encoderSim.setDistance(Hardware.simulation.getPositionMeters());
      RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(Hardware.simulation.getCurrentDrawAmps()));
      
      SmartDashboard.putNumber("Elevator Velocity", Hardware.simulation.getVelocityMetersPerSecond());
    }
    else
    {
      SmartDashboard.putNumber("Elevator Velocity", Hardware.right.getSelectedSensorVelocity());
    }
    SmartDashboard.putBoolean("Limit Switch", Hardware.limitSwitch.get());
    SmartDashboard.putString("Elevator State", move.name());
    SmartDashboard.putNumber("Setpoint", move.getValue());
    SmartDashboard.putNumber("Error", lastError);
    SmartDashboard.putNumber("Motor Output [-1, 1]", Hardware.right.getMotorOutputPercent());
    SmartDashboard.putNumber("Elevator Position", getDistance());
  }

  public double calc(Constants.MOVE move)
  {
    double error = move.getValue() - getDistance();
    double currTime = Timer.getFPGATimestamp();
    double dt = currTime - lastTime;

    errorSum += error;

    double output = Constants.kP * error + Constants.kI * errorSum + Constants.kD * (error - lastError)/(dt);

    lastError = error;
    lastTime = currTime;

    return output;
  }

  public double getDistance()
  {
    return Hardware.encoder.getDistance();
  }

  @Override
  public void periodic() 
  {
  }
}
