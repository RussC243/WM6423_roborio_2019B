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
  public void driveMotorIn()
  {
     intakeMotor.set(DRIVE_SCALE);
  }
  public void driveMotorOut()
  {
     intakeMotor.set(-DRIVE_SCALE);
  }
  public void driveMotorOff()
  {
     intakeMotor.set(0);
  }
}
