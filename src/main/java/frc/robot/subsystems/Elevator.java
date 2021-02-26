// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Hardware;

public class Elevator extends SubsystemBase {
  double totalError;
  double time;
  double prevError;
  double gravityCompensation;

  /** Creates a new Elevator. */
  public Elevator() {
    Hardware.Elevator.left = new WPI_TalonSRX(1);
    Hardware.Elevator.right = new WPI_TalonSRX(2);

    Hardware.Elevator.encoder = new Encoder(0, 1);

    Hardware.Elevator.gearbox = DCMotor.getVex775Pro(2);

    Hardware.Elevator.sim = new ElevatorSim(
      Hardware.Elevator.gearbox, 
      Constants.SimConstants.GEAR_RATIO, 
      Constants.SimConstants.MASS, 
      Constants.SimConstants.PULLEY_RADIUS, 
      Constants.SimConstants.BOTTOM, 
      Constants.SimConstants.TOP
    );

    Hardware.Elevator.limitSwitch = new DigitalInput(3);

    Hardware.Elevator.left.configFactoryDefault();
    Hardware.Elevator.right.configFactoryDefault();

    Hardware.Elevator.left.follow(Hardware.Elevator.right);

    Hardware.Elevator.encoder.reset();
    Hardware.Elevator.encoder.setDistancePerPulse(Constants.SimConstants.DISTANCE_PER_PULSE);

    Hardware.Elevator.encoderSim = new EncoderSim(Hardware.Elevator.encoder);

    time = 0;
    totalError = 0;
    prevError = 0;
    gravityCompensation = 0.1;
  }

  public double getMotorOutput(double target) {
    double error = target - Hardware.Elevator.encoder.getDistance();
    double timePassed = Timer.getFPGATimestamp() - time;

    double errorRate = (error - prevError) / timePassed;
    prevError = error;

    totalError += error * timePassed;

    double output = Constants.SimConstants.kP * error + Constants.SimConstants.kI * totalError + Constants.SimConstants.kD * errorRate /* + gravityCompensation */;

    if(output > 0)
      output = Math.min(1, output);
  
    else if(output < 1)
      output = Math.max(-1, output);
    
    prevError = error;
    time = Timer.getFPGATimestamp();

    // if(Hardware.Elevator.limitSwitch.get())
    //   return 0;

    return output;
  }

  public void setOutput(double output) {
    Hardware.Elevator.right.set(ControlMode.PercentOutput, output);
    Hardware.Elevator.sim.setInput(RobotController.getBatteryVoltage() * output);
  }

  public void stop() {
    setOutput(0);
  }

  public void simulationPeriodic() {
    super.simulationPeriodic();

    Hardware.Elevator.sim.update(0.02);
    
    Hardware.Elevator.sim.setInput(Hardware.Elevator.right.getMotorOutputVoltage());
    Hardware.Elevator.encoderSim.setDistance(Hardware.Elevator.sim.getPositionMeters());

    SmartDashboard.putNumber("Previous Error", prevError);
    SmartDashboard.putNumber("Motor Output", Hardware.Elevator.right.getMotorOutputPercent());
    SmartDashboard.putNumber("Distance Traveled", Hardware.Elevator.sim.getPositionMeters());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
