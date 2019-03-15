
/*
 * This class controls  the pneumatics and the robot's driving
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
package frc.robot;
import edu.wpi.first.wpilibj.*;
import frc.robot.HardwareMap;
import frc.robot.Robot.OurBots;

public class Pneumatics 
{
private final double PULSE_DURATION = 1.01;   // in seconds. 
private final double MOTOR_CURRENT_LIMIT = 3; // to avoid burning out a stalled motor

HardwareMap hMap; 
OurBots  selectedBots_pnuematics_local; //copy of constructor argument if needed by class methods 
Spark    pneumaticWheelLeft;         //drop wheel motor controller
Spark    pneumaticWheelRight; 
SpeedControllerGroup pnuematicGroup; //group for the two drop wheel motor controllers
Solenoid pneumatic_front_down;  //3 devices, each with 2 selenoid valves 
Solenoid pneumatic_front_up;    //- 
Solenoid pneumatic_rear_down;   //-
Solenoid pneumatic_rear_up;     //-
Solenoid pneumatic_hatch_pull;  //-
Solenoid pneumatic_hatch_push;  //-
PowerDistributionPanel panel;   // to check for motor current. 

public Pneumatics(OurBots selectedBot)//constructor
{
  selectedBots_pnuematics_local = selectedBot;      //copy to be used by other methods of this class
  hMap = new HardwareMap();                         //Create hardwareMap to access its fields. 
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
    //There are 3 pnuematic control devices, each with 2 solenoids. One extends piston and one retracts piston.
    pneumatic_front_up   = new Solenoid(hMap.canID_PCM, hMap.pnuematic_front_up); //CAN ID for PCM, position on PCM
    pneumatic_front_down = new Solenoid(hMap.canID_PCM, hMap.pnuematic_front_down); 
    pneumatic_rear_up    = new Solenoid(hMap.canID_PCM, hMap.pnuematic_rear_up);
    pneumatic_rear_down  = new Solenoid(hMap.canID_PCM, hMap.pnuematic_rear_down); 
    pneumatic_hatch_pull = new Solenoid(hMap.canID_PCM, hMap.pnuematic_hatch_pull);
    pneumatic_hatch_push = new Solenoid(hMap.canID_PCM, hMap.pnuematic_hatch_push); 
    //Set up the 6 newly created objects
    pneumatic_front_up.setPulseDuration(PULSE_DURATION); //PCM emits a pulse lasting 1.01 seconds. 
    pneumatic_front_up.startPulse();                    //pulse generated. 
    pneumatic_front_down.setPulseDuration(PULSE_DURATION);
    pneumatic_front_down.startPulse(); 
    //--
    pneumatic_rear_up.setPulseDuration(PULSE_DURATION);  
    pneumatic_rear_up.startPulse(); //pulse generated. 
    pneumatic_rear_down.setPulseDuration(PULSE_DURATION);
    pneumatic_rear_down.startPulse(); 
    //--
    pneumatic_hatch_pull.setPulseDuration(PULSE_DURATION);  
    pneumatic_hatch_pull.startPulse(); //pulse generated. 
    pneumatic_hatch_push.setPulseDuration(PULSE_DURATION);
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
public void regulate_pnuematic_MotorsCurrent()
  { /**Method for monitoring current of pneumatic motors, once it exceeds a certain value.
    We might need to stall it. */
    if(panel.getCurrent(hMap.PDP_climbWheelLeft) > MOTOR_CURRENT_LIMIT )
    {
      System.out.println("Pneumatic motor's current has exceeded 3A"); 
      //Do something. 
    }

    if(panel.getCurrent(hMap.PDP_climbWheelRight) > MOTOR_CURRENT_LIMIT )
    {
      System.out.println("Pneumatic motor's current has exceeded 3A");
      //Do something.  
    }
  } 
}
