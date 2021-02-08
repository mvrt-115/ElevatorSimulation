// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.system.plant.DCMotor;

/** Add your docs here. */
public class Hardware 
{
    public static DCMotor gearbox;
    public static ElevatorSim simulation;
    public static WPI_TalonSRX left, right;
    public static Encoder encoder;
    public static EncoderSim encoderSim;
    public static DigitalOutput limitSwitch;
}
