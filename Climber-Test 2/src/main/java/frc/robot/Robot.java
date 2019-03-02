/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
//import com.ctre.phoenix.motorcontrol
//import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Robot extends TimedRobot {
 // private static final int kMotorPort = 1;
  private static final int kJoystickPort = 0;

  private WPI_TalonSRX m_motor1, m_motor2, m_motor3;

  private Joystick m_joystick;

  private final DoubleSolenoid m_piston = new DoubleSolenoid(1, 2);


  @Override
  public void robotInit() {
    m_motor1 = new WPI_TalonSRX(1); 
    m_motor2 = new WPI_TalonSRX(2);
    m_motor3 = new WPI_TalonSRX(3);
    m_joystick = new Joystick(kJoystickPort);

    m_motor2.follow(m_motor1);
    m_motor3.follow(m_motor1);

  }



  @Override
  public void teleopPeriodic() {
    
  // m_motor1.set(m_joystick.getY());
  //  m_motor2.set(m_joystick.getY());
  //  m_motor3.set(m_joystick.getY());

    if(Math.abs(m_joystick.getY())>0.18){
      m_motor1.set(m_joystick.getY()*-0.8);
    }
    else{
      m_motor1.set(0);
      
    }
    
    if(m_joystick.getRawButton(1)) {
      m_piston.set(Value.kForward);
    }
    if(m_joystick.getRawButton(2)) {
      m_piston.set(Value.kReverse);
    }
  }


}
