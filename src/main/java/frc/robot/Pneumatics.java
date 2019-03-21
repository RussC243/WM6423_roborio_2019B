
/*
 * This class controls  the pneumatics and the robot's driving
 * when the extra wheels have been dropped down. 
 * 
 * The two pneumatic wheels will be used when the robot has elevated itself as it climbs up
 * the ramp. 
 * 
 * The front and back pneumatic systems are independent of each other. 
 * ***/

/*TODO: Need to write method to make sure the drop wheel motors do not burn out when 
  they draw too much current. 
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
      pneumatic_hatch_pull = new Solenoid(hMap.canID_PCM, hMap.pnuematic_hatch_pull);
      pneumatic_hatch_push = new Solenoid(hMap.canID_PCM, hMap.pnuematic_hatch_push); 
      //Set up the newly created objects
      pneumatic_hatch_pull.setPulseDuration(PULSE_DURATION);  
      pneumatic_hatch_pull.startPulse(); //pulse generated. 
      pneumatic_hatch_push.setPulseDuration(PULSE_DURATION);
      pneumatic_hatch_push.startPulse(); 
      break;
    }
  }//end constructor
    
  //Methods for Solenoid state. Setting them on turn them on while
  public void hatchPush()  
  {
      pneumatic_hatch_push.set(true);
      pneumatic_hatch_pull.set(false);
  }
  public void hatchPull()  
  {
      pneumatic_hatch_pull.set(true);
      pneumatic_hatch_push.set(false);
  }
    
  //place holder method... add 5 more if needed
  public void setSolenoidPulseTimes(double duration) 
  {
    //Sets the pulse duration, controlled by the PCM and triggers the PCM to make 
    //a pulse of that duration. 
    pneumatic_hatch_pull.setPulseDuration(duration);
    pneumatic_hatch_push.setPulseDuration(duration);
    pneumatic_hatch_pull.startPulse();
    pneumatic_hatch_push.startPulse();

  }
}    
