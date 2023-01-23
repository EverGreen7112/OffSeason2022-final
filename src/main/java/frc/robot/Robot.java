// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.CANifier.PWMChannel;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.EverLibEssentials.Motors;
import frc.robot.statics.Autonomus;
import frc.robot.statics.Constants;
import frc.robot.statics.Controls;
import edu.wpi.first.wpilibj.I2C;

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
  public static AHRS navX;
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    //Constants.UsableMotors.init();
    m_robotContainer = new RobotContainer();
    navX = new AHRS(I2C.Port.kMXP);
    Constants.UsableMotors.FLY_WHEEL.setSelectedSensorPosition(0);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  public static long lastTime = System.currentTimeMillis();
  public static double velocityX = 0;
  public static double x = 0;
  public static double velocityY = 0;
  public static double y = 0;
  public static double velocityZ = 0;
  public static double z = 0;
   @Override
  public void robotPeriodic() {
    //SmartDashboard.putNumber("position: ", Constants.UsableMotors.FLY_WHEEL.getSelectedSensorPosition());
    double angle = Controls.m_hubVision.getAngleX();
    SmartDashboard.putNumber("yaw", navX.getYaw());
    SmartDashboard.putNumber("pitch", navX.getPitch());
    SmartDashboard.putNumber("roll", navX.getRoll());
    SmartDashboard.putNumber("angle", navX.getAngle());
    SmartDashboard.putNumber("accel",navX.getWorldLinearAccelZ() * 9.806);

    long time = System.currentTimeMillis();
    double deltaTime = ((time - lastTime) / 1000.0);
    SmartDashboard.putNumber("delta",deltaTime);
    lastTime = time;
    
    double accelerationZ = (navX.getWorldLinearAccelZ() * 9.806);
    double accelerationX = (navX.getWorldLinearAccelX() * 9.806);
    double accelerationY = (navX.getWorldLinearAccelY() * 9.806);
    
    double deltaVelocityX = accelerationX * deltaTime; 
    double deltaX = velocityX * deltaTime + 0.5 * (accelerationX * deltaTime * deltaTime);
    velocityX += deltaVelocityX;
    x += deltaX;

    double deltaVelocityY = accelerationY * deltaTime; 
    double deltaY = velocityY * deltaTime + 0.5 * (accelerationY * deltaTime * deltaTime);
    velocityY += deltaVelocityY;
    y += deltaY;

    double deltaVelocityZ = accelerationZ * deltaTime; 
    double deltaZ = velocityZ * deltaTime + 0.5 * (accelerationZ * deltaTime * deltaTime);
    velocityZ += deltaVelocityZ;
    z += deltaZ;
       
    SmartDashboard.putNumber("x", x);
    SmartDashboard.putNumber("y", y);
    SmartDashboard.putNumber("z", z);
    SmartDashboard.putNumber("vx",velocityX);
    SmartDashboard.putNumber("vy",velocityY);
    SmartDashboard.putNumber("vz",velocityZ);

}

    //SmartDashboard.putNumber("angle2", angle);

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
  double target;
  @Override
  public void testInit() {
    //Motors.deleteRecordingsByOrder(300);
    ////SmartDashboard.putString("motor", PWMChannel.valueOf(1).name());
    // Controls.navX.reset();
    Constants.UsableMotors.FLY_WHEEL.selectProfileSlot(0, 0);
    Constants.UsableMotors.FLY_WHEEL.config_kP(0, 0.006 * 1.6);
    Constants.UsableMotors.FLY_WHEEL.config_kI(0, 0.000003);
    Constants.UsableMotors.FLY_WHEEL.config_kD(0, 0.000005);
    Constants.UsableMotors.FLY_WHEEL.setSelectedSensorPosition(0);
    // Constants.UsableMotors.FLY_WHEEL.setInverted(true);
    // Constants.UsableMotors.FLY_WHEEL.set(TalonSRXControlMode.Position, 8196);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    Constants.UsableMotors.FLY_WHEEL.setSensorPhase(true);
    Constants.UsableMotors.FLY_WHEEL.set(TalonSRXControlMode.Position, 8196*2);
    SmartDashboard.putNumber("target", Constants.UsableMotors.FLY_WHEEL.getClosedLoopTarget());
    SmartDashboard.putNumber("error", Constants.UsableMotors.FLY_WHEEL.getClosedLoopError());
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
