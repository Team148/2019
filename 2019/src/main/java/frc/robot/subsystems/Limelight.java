/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Limelight extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static Limelight m_instance;

  private double m_LEDMode = 0.0;
  private double m_pipeline = 0.0;
  private double cameraMode = 0.0;

  public Limelight() {

    super();

  //	std::cout <<"Creating Limelight" << std::endl;


    m_ledMode = 1.0;		//set LEDs to default to off
    m_pipeline = 0.0;		//sort by largest
    m_camMode = 1.0;		//vision processing ON

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    SetDefaultCommand(new UpdateLimeLight()); 
  }

  public static Limelight getInstance() {

    if (m_instance == null) {
      m_instance = new Limelight();
    }
    return m_instance;
  }

  public void getLimelightData() {
      //read values from NetworkTables
      NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
      NetworkTableEntry tv = table.getEntry("tv"); //0 or 1 for valid target
      NetworkTableEntry tx = table.getEntry("tx");
      NetworkTableEntry ty = table.getEntry("ty");
      NetworkTableEntry ledMode = table.getEntry("ledMode");
      NetworkTableEntry pipeline = table.getEntry("pipeline");
      NetworkTableEntry cammode = table.getEntry("camMode");

      //read values periodically
      double validObject = tv.getDouble(0.0);
      double xOffSet = tx.getDouble(0.0);
      double yOffSet = ty.getDouble(0.0);
      double m_LEDMode = ledMode.getDouble(0.0);
      double m_pipeline = pipeline.getDouble(0.0);
      double m_camMode = cammode.getDouble(0.0);
          
//post to smart dashboard periodically
      SmartDashboard.putNumber("HorizOffset", xOffSet);
      SmartDashboard.putNumber("validObject", validObject);

   //Write NetworkTables with desired values
      ledMode.setDouble(m_LEDMode);
      pipeline.setDouble(m_pipeline);
      cammode.setDouble(m_camMode);
  }

  public void setLimelightLEDOn() {

    m_LEDMode = 3.0;  //set permanently ON regardless of pipeline setting
  }

  public void setLimelightLEDOff() {

    m_LEDMode = 1.0;  //set permanently OFF regardless of pipeline setting
  }

  public void setLimelightLEDBlink() {

    m_LEDMode = 2.0;  //set BLINK to use as driver visual
  }

  public void setLimelightPipeline(double pipe) {

    m_pipeline = pipe;
  }

  public bool IsEnabled() {
    if(m_camMode == 0.0)
      return true;
    else
      return false;
  }
  
  
  public double GetOffsetAngle() {
 //   std::cout << xOffSet.GetDouble(0) << std::endl;
    return xOffSet.getDouble(0);
  
  }
  public double GetTargetHeading() {
    double m_target_heading = Drivetrain.getInstance().getRobotPathHeading();
    if(IsTargeting())
    {
      m_target_heading = Drivetrain.getInstance().getRobotPathHeading() + (-xOffSet.getDouble(0));
    }
    return m_target_heading;
  }
  
  public bool IsTargeting() {
    if(validObject.getDouble(0.0) > 0)
      return true;
    else
      return false;
  }
  
  void SetEnableVision(bool on) {
    if(on)
      m_camMode = 0.0;
    else
      m_camMode = 1.0;
  }
  
}
