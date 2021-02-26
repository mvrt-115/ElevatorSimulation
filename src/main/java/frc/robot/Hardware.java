package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.system.plant.DCMotor;

public class Hardware {
    public static class Elevator {
        public static WPI_TalonSRX left, right;
        public static DCMotor gearbox;
        public static ElevatorSim sim;
        public static Encoder encoder;
        public static EncoderSim encoderSim;
        public static DigitalInput limitSwitch;
    }    
}
