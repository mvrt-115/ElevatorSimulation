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
    public static final double CARRIAGE_MASS = convertPoundsToKg(15);
    public static final double GEAR_REDUCTION = 10.0;
    public static final double PULLEY_RADIUS = Units.inchesToMeters(2);
    public static final double MIN_HEIGHT = 0.0, MAX_HEIGHT = Units.inchesToMeters(11 * 12);
    public static final double TICKS_PER_ROTATION = 4096;
    public static final double Distance_PER_PULSE = 2.0 * Math.PI * PULLEY_RADIUS / GEAR_REDUCTION / TICKS_PER_ROTATION;

    public static final double kP = 1, kD = 0.4, kI = 0.004;

    public static enum MOVE 
    {
        BOTTOM(0.0), MIDDLE(2.0), TOP(3.0);

        private double value;

        private MOVE(double value)
        {
           this.value = value;
        }

		public double getValue() {
			return value;
		}
    };

    private static double convertPoundsToKg(double pounds)
    {
        return pounds * 0.45359237; 
    }
}
