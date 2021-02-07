// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ElevatorBottomCommand;
import frc.robot.commands.ElevatorMiddleCommand;
import frc.robot.commands.ElevatorTopCommand;

/** Add your docs here. */
public class OI 
{
    public OI()
    {
        SmartDashboard.putData("Elevator Bottom", new ElevatorBottomCommand());
        SmartDashboard.putData("Elevator Middle", new ElevatorMiddleCommand());
        SmartDashboard.putData("Elevator Top", new ElevatorTopCommand());
    }
}
