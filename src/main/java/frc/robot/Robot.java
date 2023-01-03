// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.CANifier.PWMChannel;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.EverLibEssentials.Motors;
import frc.robot.statics.Autonomus;
import frc.robot.statics.Constants;
import frc.robot.statics.Controls;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    //Constants.UsableMotors.init();
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    //SmartDashboard.putNumber("position: ", Constants.UsableMotors.FLY_WHEEL.getSelectedSensorPosition());
    double angle = Controls.m_hubVision.getAngleX();
    //SmartDashboard.putNumber("angle2", angle);

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() { 
    Controls.init();
    Autonomus.auto();

    // double[][] speeds = Motors.getRecordingAsDoubleArray();
    // for (int i = 0; i < speeds.length; i++) {
    //   String vals = "";
    //   for (int j = 0; j < speeds[i].length; j++) {
    //     vals += String.format(" %s", speeds[i][j]);
        
    //   }
    //   //SmartDashboard.putString("motors vals", vals);
      
    // }
    // Motors.playRecordedAction();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    Motors.runMotors();
  }

  @Override
  public void teleopInit() {
    //SmartDashboard.putNumber("target2", 0);
    Controls.init();
    SmartDashboard.putNumber("target", 0);
    Controls.m_shootPidMotor.reset();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    Controls.movePeriodic();
    Motors.runMotors();
    SmartDashboard.putNumber("ticks", Constants.UsableMotors.FLY_WHEEL.getSelectedSensorPosition());
  }

  @Override
  public void testInit() {
    //Motors.deleteRecordingsByOrder(300);
    ////SmartDashboard.putString("motor", PWMChannel.valueOf(1).name());


  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

    
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
