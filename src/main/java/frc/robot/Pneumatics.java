/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 * TODO: explain how the pnuematics work. 
*/ 



//import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.interfaces.Potentiometer;

//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.*;
//import edu.wpi.first.wpilibj.GenericHID.Hand;
//import edu.wpi.first.wpilibj.SPI.Port;
//import frc.robot.Robot;
//import frc.robot.RobotMap;
import frc.robot.HardwareMap;
import frc.robot.Robot.OurBots;

  /****
   * This class is the subsystem that controls the pneumatics and the robot's driving
   * when the extra wheels have been dropped down. 
   * 
   * The two pneumatic wheels will be used when the robot has elevated itself as it climbs up
   * the ramp. 
   * 
   * The robot should be able to come and down with the pneumatics because the front and back 
   * pneumatic systems are independent of each other. 
   * ***/
  
  /*TODO: Need to write method to make sure the pneumatic drive motors do not burn out when 
  they recieve too much current. 
  */
public class Pneumatics 
{
    HardwareMap hMap; 
    OurBots  selectedBots_pnuematics_local; 
    Spark    pneumaticWheelLeft; //Oh, the abstraction of it all!
    Spark    pneumaticWheelRight; 
    Solenoid pneumatic_front_down;  
    Solenoid pneumatic_front_up;   
    Solenoid pneumatic_rear_down;  
    Solenoid pneumatic_rear_up;    
    Solenoid pneumatic_hatch_pull; 
    Solenoid pneumatic_hatch_push; 
    PowerDistributionPanel panel; // to check for motor current.
    SpeedControllerGroup pnuematicGroup; 
    private double pulseDuration = 1.01; // in seconds. 
    private double motorCurrentThreshold = 3;

    public Pneumatics(OurBots selectedBot)//constructor
    {
      hMap = new HardwareMap();                         //Create hardwareMap to access its fields. 
      selectedBots_pnuematics_local = selectedBot;      //copy to be used by other methods of this class
      switch(selectedBot)
      {
        case PEANUT:
            //no drop-down wheels, pistons, valves, compressor, nada
            System.out.println("Peanut Bot has no pnuematic system!"); 
            break;
        case WM2019_2ND:
        case WM2019_BAG:
        default:
            //Compressor code is handled internally. Make Solenoid objects and features from there. 
            //Pneumatic Drive wheels, aka drop-down wheels. 
            pneumaticWheelRight  = new Spark(hMap.climbWheelLeft); //ID From hardware map. 
            pneumaticWheelLeft   = new Spark(hMap.climbWheelRight); 
            pnuematicGroup       = new SpeedControllerGroup(pneumaticWheelLeft, pneumaticWheelRight);
          //Pdp panel 
            panel = new PowerDistributionPanel(hMap.pdp_can_ID);

            //There are 3 pnuematic control devices, each with 2 solenoids. One extends piston and one retracts piston.
            pneumatic_front_up   = new Solenoid(hMap.pcm_can_ID, hMap.pnuematic_front_up); //CAN ID for PCM, position on PCM
            pneumatic_front_down = new Solenoid(hMap.pcm_can_ID, hMap.pnuematic_front_down); 
            pneumatic_rear_up    = new Solenoid(hMap.pcm_can_ID, hMap.pnuematic_rear_up);
            pneumatic_rear_down  = new Solenoid(hMap.pcm_can_ID, hMap.pnuematic_rear_down); 
            pneumatic_hatch_pull = new Solenoid(hMap.pcm_can_ID, hMap.pnuematic_hatch_pull);
            pneumatic_hatch_push = new Solenoid(hMap.pcm_can_ID, hMap.pnuematic_hatch_push); 
            //Set up the 6 newly created objects
            pneumatic_front_up.setPulseDuration(pulseDuration); //PCM emits a pulse lasting 1.01 seconds. 
            pneumatic_front_up.startPulse();                    //pulse generated. 
            pneumatic_front_down.setPulseDuration(pulseDuration);
            pneumatic_front_down.startPulse(); 
            //--
            pneumatic_rear_up.setPulseDuration(pulseDuration);  
            pneumatic_rear_up.startPulse(); //pulse generated. 
            pneumatic_rear_down.setPulseDuration(pulseDuration);
            pneumatic_rear_down.startPulse(); 
            //--
            pneumatic_hatch_pull.setPulseDuration(pulseDuration);  
            pneumatic_hatch_pull.startPulse(); //pulse generated. 
            pneumatic_hatch_push.setPulseDuration(pulseDuration);
            pneumatic_hatch_push.startPulse(); 
            break;
        }
    }//end constructor
      
    //Methods for Solenoid state. Setting them on turn them on while
    public void frontExtend()  
    {
        pneumatic_front_up.set(true);
        pneumatic_front_down.set(false);
    }
    public void frontRetract()  
    {
        pneumatic_front_down.set(true);
        pneumatic_front_up.set(false);
    }
    public void rearExtend()  
    {
        pneumatic_rear_up.set(true);
        pneumatic_rear_down.set(false);
    }
    public void rearRetract()  
    {
        pneumatic_rear_down.set(true);
        pneumatic_rear_up.set(false);
    }
    public void hatchExtend()  
    {
        pneumatic_hatch_push.set(true);
        pneumatic_hatch_pull.set(false);
    }
    public void hatchRetract()  
    {
        pneumatic_hatch_pull.set(true);
        pneumatic_hatch_push.set(false);
    }
      
    //place holder method... add 5 more if needed
    public boolean isFrontUpOn() //Checks to see if the front solenoid is on. 
    {
      if(pneumatic_front_up.get() == true)
      {
        System.out.println("Front Up solenoid is on.");
        return true; 
      } 
      return false; 
    }
   
    public void driveDropWheels(double value) 
    { 
       pnuematicGroup.set(value);
    }

    public void regulate_pnuematic_MotorsCurrent()
    { /**Method for monitoring current of pneumatic motors, once it exceeds a certain value.
      We might need to stall it. */
      if(panel.getCurrent(hMap.climbWheelLeft) > motorCurrentThreshold )
      {
        System.out.println("Pneumatic motor's current has exceeded 3A"); 
        //Do something. 
      }

      if(panel.getCurrent(hMap.climbWheelRight) > motorCurrentThreshold )
      {
        System.out.println("Pneumatic motor's current has exceeded 3A");
        //Do something.  
      }
    } 

    public void setSolenoidPulseTimes(double duration) 
    {
        //Sets the amount of seconds of the pulse is controlled by the PCM and triggers the PCM to make 
        //a pulse of the duration passsed in the 'duration' parameter of this method. 
        pneumatic_front_down.setPulseDuration(duration);
        pneumatic_front_up.setPulseDuration(duration);
        pneumatic_rear_down.setPulseDuration(duration);
        pneumatic_rear_up.setPulseDuration(duration);
        pneumatic_hatch_pull.setPulseDuration(duration);
        pneumatic_hatch_push.setPulseDuration(duration);
        pneumatic_front_down.startPulse();
        pneumatic_front_up.startPulse();
        pneumatic_rear_down.startPulse();
        pneumatic_rear_up.startPulse();
        pneumatic_hatch_pull.startPulse();
        pneumatic_hatch_push.startPulse();
  }    

}