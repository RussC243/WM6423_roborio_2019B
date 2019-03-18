package frc.robot;
import edu.wpi.first.wpilibj.*;
/**
 * handles the Intake 
*/
public class Intake 
{
  HardwareMap hMap;
  Spark  intakeMotor;
  double DRIVE_SCALE = 1.00;
  
  public Intake()//constructor
  {
    hMap = new HardwareMap();
    intakeMotor = new Spark(hMap.motorIntake);
  }
  public void driveMotorIn(double motorDriveValue)
  {
     if(Math.abs(motorDriveValue)>0.1) //don't creep
     {
        intakeMotor.set(DRIVE_SCALE * motorDriveValue);
     }
     else
     {
        intakeMotor.set(0);
     }
  }
}
