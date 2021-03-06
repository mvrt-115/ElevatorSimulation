// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private ElevatorSim m_elevatorSim;
  private WPI_TalonSRX left, right;

  private Encoder encoder;
  private EncoderSim m_encoderSim;
  double motorOutput;

  double lastTime = 0;
  double lastError = 0;
  double intergral = 0;
  double totalError = 0;

  double lastOutput = 0;

  double setPoint = 0;

  public enum ElevatorStates{
    ZERO, SETPOINT, DISABLED;
  }

  private ElevatorStates state;

  /** Creates a new Elevator. */
  public Elevator() {
    m_elevatorSim = new ElevatorSim(Constants.Elevator.motors, 
                                    Constants.Elevator.kGearRatio, 
                                    Constants.Elevator.kCarriageMass, 
                                    Constants.Elevator.kPulleyRadius, 
                                    Constants.Elevator.kMinHeight, 
                                    Constants.Elevator.kMaxHeight);
    
    encoder = new Encoder(0, 1);
    m_encoderSim = new EncoderSim(encoder);


    left = new WPI_TalonSRX(1);
    right = new WPI_TalonSRX(2);

    left.configFactoryDefault();
    right.configFactoryDefault();

    left.follow(right);

    state = ElevatorStates.DISABLED;
  }

  public void setState(ElevatorStates newState) {
    state = newState;
  }

  public void setSetpoint(double setpoint) {
    setPoint = setpoint;
  }

  public double calculateToSetpoint() {
    double error = setPoint - m_elevatorSim.getPositionMeters();
    double dt = Timer.getFPGATimestamp();
    intergral += error;
    double errorRate = (error - lastError)/ dt;

    if(lastOutput > 0)
      lastOutput = Math.min(1, lastOutput);
    
    else if(lastOutput < 1)
      lastOutput = Math.max(-1, lastOutput);
    
    motorOutput = Constants.Elevator.kP * error + Constants.Elevator.kI * intergral + Constants.Elevator.kD * errorRate;
    
    lastError = error;
    lastTime = Timer.getFPGATimestamp();
    lastOutput = motorOutput;
    
    if(setPoint == Constants.Elevator.kBottomSetpoint && m_elevatorSim.getPositionMeters() <= 0.05){
      state = ElevatorStates.ZERO;
    }
    return motorOutput;
  }

  public void move(double speed){
    right.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();
    m_elevatorSim.setInput(right.getMotorOutputPercent()* RobotController.getBatteryVoltage());
    m_elevatorSim.update(.02);
  }

  public void log(){
    SmartDashboard.putNumber("Motor Output [-1, 1]", motorOutput);
    SmartDashboard.putNumber("Elevator Position", m_elevatorSim.getPositionMeters());
    SmartDashboard.putNumber("Elevator Velocity", m_elevatorSim.getVelocityMetersPerSecond());
    SmartDashboard.putNumber("Setpoint", setPoint);
    SmartDashboard.putNumber("Error", setPoint - m_elevatorSim.getPositionMeters());

    
  }

  @Override
  public void periodic() {
    switch (state) {
      case SETPOINT:
        SmartDashboard.putString("Elevator State", "SETPOINT");
        double speed = calculateToSetpoint();
        move(speed);
        break;

      case ZERO:
        SmartDashboard.putString("Elevator State", "ZEROED");
        move(0);
        break;

      case DISABLED:
        SmartDashboard.putString("Elevator State", "DISABLED");
        move(0);
        break;
    }

    log();
  }
}
