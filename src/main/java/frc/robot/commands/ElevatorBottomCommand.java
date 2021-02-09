// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Constants;
import frc.robot.Hardware;

public class ElevatorBottomCommand extends CommandBase {
  /** Creates a new ElevatorBottomCommand. */
  public ElevatorBottomCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.getElevator());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.getElevator().move(Constants.MOVE.BOTTOM);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putNumber("Error", 0);
    SmartDashboard.putBoolean("Limit Switch", Hardware.limitSwitch.get());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Hardware.limitSwitch.get();
  }
}
