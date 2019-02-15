// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc3397.UnamedRobot.subsystems;

import org.usfirst.frc3397.UnamedRobot.OI;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.Solenoid;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class Hatch {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private VictorSP hatchArm;
    private Solenoid hatchRam;
    private OI controlScheme = new OI(0, 1);
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public Hatch() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        hatchArm = new VictorSP(5);
        hatchRam = new Solenoid(0, 0);
        
        
        
        

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }
    public void HatchWork() {
        if (controlScheme.pushHatch() || controlScheme.getX()){
            hatchRam.set(true);
        } else if (controlScheme.pullHatch()){
            hatchRam.set(false);
        }
    }
    public void HatchArm() {
        if (controlScheme.getHatchArmDown() || controlScheme.getB()){
            hatchArm.setSpeed(.5);
        } else {
            hatchArm.setSpeed(-.3);
        }
        
    }
    public void HatchIn() {
        hatchRam.set(false);
    }
    
    public void HatchOut() {
        hatchRam.set(true);
    }
   
}
