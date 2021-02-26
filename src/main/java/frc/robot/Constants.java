// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class SimConstants {
        public static final double MASS = poundsToKg(15);
        public static final double BOTTOM = 0;
        public static final double MIDDLE = Units.feetToMeters(6);
        public static final double TOP = Units.feetToMeters(11);
        public static final double GEAR_RATIO = 10;
        public static final double PULLEY_RADIUS = Units.inchesToMeters(2);
        public static final double kP = 1;
        public static final double kI = 0.004;
        public static final double kD = 0.4;
        public static final double DISTANCE_PER_PULSE = 2.0 * Math.PI * PULLEY_RADIUS / GEAR_RATIO / 4096;

        public static double poundsToKg(double pounds) {
            return pounds / 2.205;
        }
    }
    
}