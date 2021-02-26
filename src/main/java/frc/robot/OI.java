package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ElevatorGoToTarget;

public class OI {
    public OI() {
        SmartDashboard.putData("Go to bottom", new ElevatorGoToTarget(Constants.SimConstants.BOTTOM));
        SmartDashboard.putData("Go to middle", new ElevatorGoToTarget(Constants.SimConstants.MIDDLE));
        SmartDashboard.putData("Go to top", new ElevatorGoToTarget(Constants.SimConstants.TOP));
    }
}
