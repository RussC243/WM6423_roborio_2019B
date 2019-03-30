package frc.robot;
import edu.wpi.first.wpilibj.*;
/**
 * handles the Intake 
*/
public class Intake 
{
  HardwareMap hMap;
  Spark  intakeMotor;
  double DRIVE_SCALE_IN = 0.7;
  double DRIVE_SCALE_OUT = 1;
  
  public Intake()//constructor
  {
    hMap = new HardwareMap();
    intakeMotor = new Spark(hMap.motorIntake);
  }
  public void driveMotorIn()
  {
     intakeMotor.set(DRIVE_SCALE_IN);
  }
  public void driveMotorOut()
  {
     intakeMotor.set(-DRIVE_SCALE_OUT);
  }
  public void driveMotorOff()
  {
     intakeMotor.set(0);
  }
}
