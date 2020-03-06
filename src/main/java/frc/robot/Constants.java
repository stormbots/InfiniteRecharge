/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.stormbots.PiecewiseLerp;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public class Constants {
    public enum BotName {COMP,PRACTICE,TABI};
    public static BotName botName = BotName.COMP;

    /** Convert Inches to Meter */
    public static double INCHES_TO_METERS = 0.0254;

    /** Initial compass bearing of the robot on boot. Potentially useful. Set in  */
    public static double INITIAL_COMPASS_HEADING = 0;

    //Measured at Practice Field - 99 is correct
    public static double VISION_TARGET_HEIGHT = 99;//92;
    // public static double VISION_DESK_CLERK_TARGET_HEIGHT = 46.5;

    /** Height of the camera, in inches */
    public static double CAMERA_MOUNT_HEIGHT = 16.25;

    /**Mount tilt back, in degrees*/
    //what? public static double CAMERA_MOUNT_ANGLE = 27.69; //13.64 prev
    public static double CAMERA_MOUNT_ANGLE = 6.58; //(untested)
    
    /** Diameter of the shooter wheel, in inches*/
    public static double SHOOTER_WHEEL_DIAMETER=4;

    /** Angle the shooter shoots the balls */
    public static double SHOOTER_ANGLE = 50;

    /** Height from the floor to the center of the shooter */
    public static double SHOOTER_HEIGHT = 22.0;

    public static double CHASSIS_TURN_STATIC_FF=0.035;

    public static PiecewiseLerp distanceToRPM;

    public static double shotFudgeFactor = 1;
      
    /** In case we need to manage robot differences, we can do so here.*/
    public static void Initialize(){
        /* COMPBOT */
        if(botName==BotName.COMP){
            distanceToRPM =  new PiecewiseLerp(  
                new double[]{0,    8.75*12, 10*12,  11*12,  12*12, 14*12, 16*12, 18*12, 20*12, 22*12}, 
                new double[]{6000, 6225,    5400,   5450,   5475,  5475,  5475,  5475,  5475,  5525} //8.7512 (105) was 6050
                //NOTE: End values are just to constrain the system, and are not viable shot values
             );         
        /* PRACTICEBOT */
        }else{
            distanceToRPM =  new PiecewiseLerp(  
               new double[]{13*12, 15*12, 17*12, 19*12, 21*12, 23*12}, 
            //    new double[]{4450,  4700,   5000, 5350,   5650, 6000}//actual values
               new double[]{1000,  1200,   1400, 1400,   2000, 2000}//plz be quiet
            );         
        }
        for(int i=0;i < distanceToRPM.inputs.length;i++) distanceToRPM.inputs[i]*=shotFudgeFactor;
        for(int i=0;i < distanceToRPM.outputs.length;i++) distanceToRPM.outputs[i]*=shotFudgeFactor;


    }
}
