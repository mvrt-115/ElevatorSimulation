// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Units;

/** Add your docs here. */
public class Constants {
    public static class Elevator {
    
        public static final double kGearRatio = 10.0;
        public static final double kPulleyRadius = Units.inchesToMeters(2);
        public static final double kCarriageMass = 6.803;
        public static final double kMinHeight = 0;
        public static final double kMaxHeight = 3.35; 
        public static final double kTicksPerRevolution = 4096;
        public static final double kDistancePerTick = 2 * Math.PI * kPulleyRadius / kTicksPerRevolution / kGearRatio;
        public static final double kP  = 0.5;
        public static final double kI = 0.005;
        public static final double kD = 1.5;
        public static final double kBottomSetpoint = 0;
        public static final double kMidSetpoint = 2;
        public static final double kHightSetpoint = 3;
        public static final DCMotor motors = DCMotor.getVex775Pro(2);
    }
}
