// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc3397.Aurora2019.subsystems;

import com.kauailabs.navx.frc.AHRS;

import org.usfirst.frc3397.Aurora2019.Constants;
import org.usfirst.frc3397.Aurora2019.OI;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SPI;
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveTrain {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private Victor rightRear;
    private Victor rightFront;
    private Victor leftFront;
    private Victor leftRear;
    private MecanumDrive chassis;
    private OI controlScheme = new OI(0, 1);
    private AnalogInput ultra = new AnalogInput(0);
    private NetworkTable table;
    Timer switchTimer;
    Timer experimentalTimer;
    boolean hatchDriveMode;
    int numberOfTimes;
    AHRS ahrs;
    double spaceBetween;
    double spaceLeft;
    double middleLeft;
    double middleRight;
    double leftDistance;
    double rightDistance;
    double distanceDifference;
    double distanceRT;
    double targetDistance;
    double distanceError;
    double visionForward;
    double visionTurn;
    double width;
    double cwidth;
    double gyroAngle;
    PIDController controller;
    double currentYaw;
    double oldYaw;
    double elapsedYaw;
    double yawAngle;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public DriveTrain() {
        System.out.println("DriveTrain started");
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        rightRear = new Victor(Constants.backRightMotorPWM);
        rightFront = new Victor(Constants.frontRightMotorPWM);
        leftFront = new Victor(Constants.frontLeftMotorPWM);
        leftRear = new Victor(Constants.backLeftMotorPWM);
        chassis = new MecanumDrive(leftFront, leftRear,
              rightFront, rightRear);
        
            chassis.setSafetyEnabled(true);
            chassis.setExpiration(0.1);
            chassis.setMaxOutput(1.0);
        rightRear.set(.4);
        hatchDriveMode = false;
        numberOfTimes = 1;
        ahrs = new AHRS(SPI.Port.kMXP); 
        switchTimer = new Timer();
        switchTimer.start();
        experimentalTimer = new Timer();
        experimentalTimer.start();
        NetworkTable.setIPAddress("10.33.97.2");
        table = NetworkTable.getTable("GRIP");
        oldYaw = ahrs.getYaw();
        
        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }
    public void defaultCargoFront(){
        hatchDriveMode = false;
    }
    public void MecanumDrive() {
        
        double forward;
        double strafe;
        double turn;

        // MyTimer timeThis;

        // if (buttonPressed && (timeThis == null || timeThis.isTimeUp())) {
        //     timeThis = new MyTimer(5);
        // }



        if (controlScheme.switchFrontNBack() && switchTimer.hasPeriodPassed(1.0)){
            switchTimer.reset();
            System.out.println("Switching to HatchDriveMode");
            hatchDriveMode = !hatchDriveMode;
        } 
        if (hatchDriveMode){
            forward = -controlScheme.getDriveLeftX();
            strafe = controlScheme.getDriveLeftY();
            turn = controlScheme.getDriveRightX();
        } else {
            forward = controlScheme.getDriveLeftX();
            strafe = -controlScheme.getDriveLeftY();
            turn = -controlScheme.getDriveRightX();
        }
        
         
        double turnSpeed = 0.5;
        double speedMultiplier = 0.5;
        double deadzone = 0.2;
       
        if (Math.abs(forward) <= deadzone) {
            forward = 0.0;
        }
        
        if (Math.abs(strafe) <= deadzone) {
            strafe = 0.0;
        }
        if (Math.abs(turn) <= deadzone) {
            turn = 0.0;
        }
        
        if (Math.abs(turn) >= deadzone) {
            turn *= -0.8;
           
            
            chassis.driveCartesian(0.0,  0.0,  turn, gyroAngle);
            
        }

        else if ((Math.abs(forward) >= deadzone || Math.abs(strafe) >= deadzone))
        { 
            
//			
            
            
            strafe *= 1;
            forward *= .95;
            
            
			chassis.driveCartesian(forward, strafe, 0.0);
//			frontLeftMotor.set(forward);
//			frontRightMotor.set(-forward);
//			backLeftMotor.set(forward);
//			backRightMotor.set(-forward);
            
            
        }
        else
        {
            chassis.driveCartesian(0.0, 0.0, 0.0);
        }
        // double[] defaultarray = null;
        // double[] centerX = table.getSubTable("myContoursReport").getNumberArray("centerX", defaultarray);
        // double[] height = table.getSubTable("myContoursReport").getNumberArray("height", defaultarray);
        // double[] width = table.getSubTable("myContoursReport").getNumberArray("width", defaultarray);
        
        //     if (((centerX.length > 1) && (centerX.length < 3)) && controlScheme.getFSAButton()) {
        //         SmartDashboard.putNumber("CenterX1", centerX[0]);
        //         SmartDashboard.putNumber("CenterX2", centerX[1]); 
        //         // SmartDashboard.putNumber("Height Ratio", height[0]/height[1]);
        //         // SmartDashboard.putNumber("Width Ratio", width[0]/width[1]);
        //         spaceBetween = Math.abs(centerX[0] - centerX[1]);
        //         SmartDashboard.putNumber("spaceBetween (Pixels)", spaceBetween);
        //         distanceRT = (707.1348-31.05719*spaceBetween)+(0.5704162*spaceBetween*spaceBetween)-(0.004762775*spaceBetween*spaceBetween*spaceBetween)+(0.00001494933*spaceBetween*spaceBetween*spaceBetween*spaceBetween);
        //         SmartDashboard.putNumber("Camera-Target Distance (Inches)", distanceRT);
        //         distanceError = distanceRT - targetDistance;
           
        //         targetDistance = SmartDashboard.getNumber("Distance Chooser", 50);
                
               
        //         if (!(Math.abs(distanceError) <= 5)){
        //             System.out.println(targetDistance);
        //             if (distanceError > 0 && distanceError < 30){
        //                 visionForward = .2;
        //             } else if (distanceError > 30 && distanceError < 50){
        //                 visionForward = .3;
        //             }  else if (distanceError < 50){
        //                 visionForward = .35;
        //             }
        //             if (distanceError < 0 && distanceError > -30){
        //                 visionForward = -.2;
        //             } else if (distanceError < -30 && distanceError > -50){
        //                 visionForward = -.3;
        //             } else if (distanceError < -50){
        //                 visionForward = -.35;
        //             }
        //         }
             
        //         if (turn == 0.0 && forward == 0.0) {
                    
        //             if (centerX[0] > centerX[1]){
        //                 middleLeft = centerX[1];
        //                 middleRight = centerX[0];
        //             } else {
        //                 middleLeft = centerX[0];
        //                 middleRight = centerX[1]; 
        //             }
        //             leftDistance = middleLeft;
        //             rightDistance = 320 - middleRight;
        //             distanceDifference = leftDistance - rightDistance;
        //             SmartDashboard.putNumber("distanceDifference", distanceDifference);
                   
                  
                    
        //             if (!(Math.abs(distanceDifference) <= 5)){
        //                 System.out.println(",");
        //                 if (distanceDifference > 5 && distanceDifference < 15){
        //                     visionTurn = -.15;
        //                 }
        //                 else if (distanceDifference > 15 && distanceDifference < 25){
        //                     visionTurn = -.19;
        //                 }
        //                 else if (distanceDifference > 25){
        //                     visionTurn = -.19;
        //                 }
        //                 if (distanceDifference < -5 && distanceDifference > -25){
        //                     visionTurn = .15;
        //                 }
        //                 else if (distanceDifference < -15 && distanceDifference > -25){
        //                     visionTurn = .17;
        //                 }
        //                 else if (distanceDifference < -25){
        //                     visionTurn = .19;
        //                 }
                
        //         }
        //         System.out.println("Vision Forward is " + visionForward);
        //         System.out.println("Vision Turn is " + visionTurn);
        //         chassis.driveCartesian(0, visionForward, visionTurn);
        //        visionForward = 0;
        //        visionTurn = 0; 
        //     }
        // }
        yawAngle = ahrs.getAngle();
        if (yawAngle > 360 || yawAngle < -360){
            yawAngle = 0;
        }
        SmartDashboard.putNumber("Yaw Angle", yawAngle);
       
        gyroAngle = ahrs.getRawGyroZ();
        // if (controlScheme.getFSBButton() && experimentalTimer.get() > 3){
        //     experimentalTimer.reset();
        //     while (experimentalTimer.get() < 1){
        //         Turn(0.3);
                
        //     }
        //     currentYaw = ahrs.getYaw();
        //         elapsedYaw = currentYaw - oldYaw;
        //         oldYaw = currentYaw;
        //         System.out.println(elapsedYaw);
            
            
        // }
    }
    public void Drive(double speed, double gyroAngle) {
        
        chassis.driveCartesian(0.0, speed, 0.0, gyroAngle);
       
    }
    
    public void stop() {
        chassis.driveCartesian(0.0, 0.0, 0.0);
    }
    
    public void Strafe(double speed) {
        speed *= -1;
        chassis.driveCartesian(0, speed, 0);
    }
    public void Turn(double speed) {
       
        chassis.driveCartesian(0, 0, speed);
        
    }
    public void runRightRear(int time) throws Exception {
        rightRear.set(.3);
        Thread.sleep(time*1000);
        rightRear.set(0);
    }
   
    public void runLeftRear(int time) throws Exception {
        leftRear.set(-.3);
        Thread.sleep(time*1000);
        leftRear.set(0);
    }
   
    public void runRightFront(int time) throws Exception {
        rightFront.set(.3);
        Thread.sleep(time*1000);
        rightFront.set(0);
    }
   
    public void runLeftFront(int time) throws Exception {
        leftFront.set(-.3);
        Thread.sleep(time*1000);
        leftFront.set(0);
    }
    public void rightRearMotor(double speed){
        rightRear.set(speed);
    }
    public void rightFrontMotor(double speed){
        rightFront.set(speed);
    }
    public void leftRearMotor(double speed){
        leftRear.set(speed);
    }
    public void leftFrontMotor(double speed){
        leftFront.set(speed);
    }
   
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

}


