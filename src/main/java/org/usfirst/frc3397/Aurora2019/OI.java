// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc3397.Aurora2019;


import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;



public class OI {
    
    public XboxController driveStick;
    public Joystick operatorStick;

   

    public OI(int drivePort, int operatePort) {
        

        operatorStick = new Joystick(operatePort);
        
        driveStick = new XboxController(drivePort);
        


       
    }
    //These are to get the buttons to use in other places 

    public double getDriveLeftY() {
		return driveStick.getY(Hand.kLeft);
	}
	
	public double getDriveLeftX() {
		return driveStick.getX(Hand.kLeft);
	}
	
	public double getDriveRightX() {
		return driveStick.getX(Hand.kRight);
    }
    public boolean getOperatorAxis3() {
        if (operatorStick.getRawAxis(3) > 0){
            return true;
        } else {
            return false;
        }
        
    }
    public boolean getOperatorAxis2() {
        if (operatorStick.getRawAxis(2) > 0){
            return true;
        } else {
            return false;
        }
    }
    public boolean getRightTrigger() {
        if (driveStick.getTriggerAxis(Hand.kRight) > 0.0){
            return true;
        } else {
            return false;
        }
		
	}
	
	public boolean getLeftTrigger() {
        if (driveStick.getTriggerAxis(Hand.kLeft) > 0.0){
            return true;
        } else {
            return false;
        }
    }
    public boolean getIntakeArmDown() {
        if (operatorStick.getRawAxis(1) > 0){
            return true;
        } else {
            return false;
        }
    }
    public boolean getIntakeArmUp() {
        if (operatorStick.getRawAxis(1) < 0){
            return true;
        } else {
            return false;
        }
    }
    public boolean getHatchArmDown() {
        return operatorStick.getRawButton(1);
    }
 
    public boolean getX() {
		return driveStick.getXButton();
	}
	
	public boolean getB() {
		return driveStick.getBButton();
	}
	public boolean getY() {
		return driveStick.getYButton();
	}
	public boolean getA() {
		return driveStick.getAButton();
    }
    public boolean switchFrontNBack() {
        return driveStick.getAButton();
    }
    public boolean getLBButton() {
        return driveStick.getRawButton(5);
    }
    public boolean getRBButton() {
        return driveStick.getRawButton(6);
    }
    public boolean getFSAButton(){
        return operatorStick.getRawButton(1);
    }
    public boolean getFSBButton(){
        return operatorStick.getRawButton(2);
    }
    public boolean intakeIn(){
        return operatorStick.getRawButton(5);
    }
    public boolean intakeOut(){
        return operatorStick.getRawButton(6);
    }
    public boolean operateLittleArms(){
        return operatorStick.getRawButton(1);
    }
    public boolean operatorX(){
        return operatorStick.getRawButton(3);
    }
    public boolean operatorY(){
        return operatorStick.getRawButton(4);
    }
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
    public XboxController getDriveStick() {
        return driveStick;
    }

    public Joystick getOperatorStick() {
        return operatorStick;
    }

    public void vibrate() {
		driveStick.setRumble(RumbleType.kLeftRumble, 1);
		driveStick.setRumble(RumbleType.kRightRumble, 1);
	}
	
	public void vibrateOff() {
		driveStick.setRumble(RumbleType.kLeftRumble, 0);
		driveStick.setRumble(RumbleType.kRightRumble, 0);
	}


   
}
