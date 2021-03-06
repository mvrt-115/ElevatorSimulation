package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.MoveToSetpoint;

public class OI {
  
    public static final Joystick joystick = new Joystick(0);
    public static final JoystickButton setPointBottomButton = new JoystickButton(joystick, 1); 
    public static final JoystickButton setPointMiddleButton = new JoystickButton(joystick, 2); 
    public static final JoystickButton setPointHighButton = new JoystickButton(joystick, 3); 
    
    public OI(){
        setPointBottomButton.whenPressed(new MoveToSetpoint(Constants.Elevator.kBottomSetpoint));
        SmartDashboard.putData("Bottom", new MoveToSetpoint(Constants.Elevator.kBottomSetpoint));
        setPointMiddleButton.whenPressed(new MoveToSetpoint(Constants.Elevator.kMidSetpoint));
        SmartDashboard.putData("Middle", new MoveToSetpoint(Constants.Elevator.kMidSetpoint));
        setPointHighButton.whenPressed(new MoveToSetpoint(Constants.Elevator.kHightSetpoint));
        SmartDashboard.putData("High", new MoveToSetpoint(Constants.Elevator.kHightSetpoint));
    }

    public static boolean getSetPointBottomButton(){
        return setPointBottomButton.get();
    }

    public static boolean getSetPointMiddleButton(){
        return setPointMiddleButton.get();
    }

    public static boolean getSetPointHighButton(){
        return setPointHighButton.get();
    }
    
}
