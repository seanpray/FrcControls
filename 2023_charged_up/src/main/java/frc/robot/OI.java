package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class OI {

    /*
        * ON LOGITECH F310 CONTROLLER:
        * X = 1            (Blue)
        * A = 2            (Green)
        * B = 3            (Red)
        * Y = 4            (Yellow)
        * LB = 5           (Left-Bumper: top button)
        * RB = 6           (Right-Bumper: top button)
        * LT = 7           (Left-Trigger: bottom button)
        * RT = 8           (Right-Trigger: bottom button)
        * Select/Back = 9  (Above left joystick)
        * Start = 10       (Above right joytsick)
        * L3 = 11          (Press left joystick)
        * R3 = 12          (Press right joystick)
        * 
        * Left Joystick Vertical Axis = 1
        * Left Joystick Horizontal Axis = 0
        * Right Joystick Vertical Axis = 3
        * Right Joystick Horizontal Axis = 2
    */

    /*  
        XBOX CONTROLLER BINDINGS
        
        Axis 1: Left Thumbstick Left-Right
        Axis 4: Right Thumbstick Left-Right
        Axis 2: Left Thumb Stick Up-Down(Needs to be negated or else up-down controls are inverted)
        Axis 5: Right Thumb Stick Up-Down(Needs to be negated or else up-down controls are inverted)
        Axis 3(>0): Left Trigger
        Axis 3(<0): Right Trigger

        1: A Button
        2: B Button
        3: X Button
        4: Y Button
        5: Left Bumper
        6: Right Bumper
        7: Stop/Back Button
        8: Start Button
        9: Left Thumbstick Button
        10: Right Thumbstick Button

        DPAD:
        -1: No Thumbpad Button
        0: North Thumbpad Button
        45: North-East Thumbpad Button
        90: East Thumbpad Button
        135: South-East Thumbpad Button
        180: South Thumbpad Button
        225: South-West Thumbpad Button
        270: West Thumbpad Button
        315: North-West Thumbpad Button
     */

    // Driver joystick object initialization
    public XboxController driver = new XboxController(Constants.driver);

    /**
     * Gets the value of an input joystick axis
     * @param axis Axis of the joystick to get
     * @return Value of axis
     */
    public double GetDriverRawAxis(int axis){
        
        return driver.getRawAxis(axis);
        
    }

    /**
     * Get the Y value of the left stick of the driver joystick
     * @return Y value of driver left stick 
     */
    public double getDriverY(){
        return driver.getLeftY();
    }

    /**
     * Get the X value of the right stick of the driver joystick
     * @return X value of driver right stick 
     */
    public double getDriverZ(){
        return driver.getRightX();
    }
    

    // Operator joystick object initialization
    public XboxController operator = new XboxController(Constants.operator);


    /**
     * Gets whether the operator has a given butotn pressed
     * @param button index of the button to check
     * @return Boolean whether the button is pressed
     */
    public boolean getOperatorButton(int button) {
        return operator.getRawButton(button);
    }



}
