package frc.robot.statics;

import java.util.LinkedList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.EverLibEssentials.Motor;
import frc.robot.EverLibEssentials.Motors;

public class Controls {
    private static Joystick m_rightJoystick  = new Joystick(Constants.JoystickPorts.rightJoystick), m_leftJoystick, m_operator; 
	private static JoystickButton m_cancelAll;
  private static Trigger m_fire;
  private static JoystickButton m_collectorCollect, m_collectorUncollect, m_collectorOpen, m_collectorClose, m_climberDown,
   m_climberUp, m_storageUp, m_storageDown, m_load, m_turnToShoot,m_driveStright, m_turbu = new JoystickButton(m_rightJoystick, 1);

   public static Vision m_hubVision = new Vision(Constants.ComPorts.hubVision);
   private static PIDMotor m_shootPidMotor = new PIDMotor(Constants.UsableMotors.FLY_WHEEL,
     Constants.PIDValues.FLY_WHEEL_KP,
     Constants.PIDValues.FLY_WHEEL_KI,
     Constants.PIDValues.FLY_WHEEL_KD);
     
    

  public static void init() {
  
    // initialize joysticks and operator
    // m_rightJoystick 
    m_leftJoystick = new Joystick(Constants.JoystickPorts.leftJoystick);
    m_operator = new Joystick(Constants.JoystickPorts.operator);
    m_driveStright = new JoystickButton(m_rightJoystick, 4);
    // m_turbo = new JoystickButton(m_rightJoystick, Constants.ButtonPorts.turbo);
	  // m_cancelAll =   new JoystickButton(m_rightJoystick,5);
    // initialize buttons for specific commands later on
    m_collectorOpen = new JoystickButton(m_operator, Constants.ButtonPorts.collectorOpen);
    m_collectorClose = new JoystickButton(m_operator, Constants.ButtonPorts.collectorClose);
    m_collectorCollect = new JoystickButton(m_operator, Constants.ButtonPorts.collectorCollect);
    m_collectorUncollect = new JoystickButton(m_operator, Constants.ButtonPorts.collectorUncollect);


    m_climberDown = new JoystickButton(m_operator, Constants.ButtonPorts.climberDown);
    m_climberUp = new JoystickButton(m_operator, Constants.ButtonPorts.climberUp);

    m_storageUp = new JoystickButton(m_operator, Constants.ButtonPorts.storageUp);
    m_storageDown = new JoystickButton(m_operator, Constants.ButtonPorts.storageDown);

    m_turnToShoot =  new JoystickButton(m_rightJoystick, Constants.ButtonPorts.turnToShoot);

    m_load = new JoystickButton(m_operator, Constants.ButtonPorts.load);
    m_fire = new POVButton(m_operator, 0);
    for (int i =0; i <= 360; i++){
      m_fire = m_fire.or(new POVButton(m_operator, i));
    }
    turnPID.setSetpoint(0);
    Constants.UsableMotors.CHASSIS_RIGHT.setInverted(true);
    Constants.UsableMotors.COLLECTOR_COLLECT.setInverted(true);
    Constants.UsableMotors.BOTTOM_STORAGE.setInverted(true);
    Constants.UsableMotors.STORAGE_TOP.setInverted(true);
    Constants.UsableMotors.FLY_WHEEL.configFactoryDefault();
    // initialize commands on buttons

  }
  // public static JoystickButton getM_turbu() {
  //     return m_turbu;
  // }

  public static void driveByJoysticks() {
    Vector2d speedVector = new Vector2d(m_rightJoystick.getY() * Constants.Speeds.moveSpeed,
     m_leftJoystick.getY() * Constants.Speeds.moveSpeed);
     double magnitude = speedVector.magnitude();
    if ( magnitude > Constants.Speeds.speedLimit){
      speedVector.x = (speedVector.x / magnitude) *  Constants.Speeds.speedLimit;
      speedVector.y = (speedVector.y / magnitude) *  Constants.Speeds.speedLimit;
    }
    if(!m_turbu.get()){ 
    tankDrive(speedVector.x, speedVector.y);
    }
    else{
      tankDrive(m_rightJoystick.getY(),  m_leftJoystick.getY() );
    }
  }
  
  public static void tankDrive(double speedRight, double speedLeft){
    Constants.UsableMotors.CHASSIS_RIGHT.set(speedRight);
    Constants.UsableMotors.CHASSIS_LEFT.set(speedLeft);
  }
  public static boolean finished2 = false;
  public static void driveStright(){
  
    double dist = m_hubVision.getZ();
    b.add(dist);
    // if(b.size() > 5){ 
    // double myAng = b.getLast() + b.get(b.size()-2) + b.get(b.size() -3);
    // dist = myAng/3;
    // }

    // double speed = turnPID.calculate(angle,0);
    //SmartDashboard.putNumber(*"turnSpeed", speed);
    //SmartDashboard.putNumber("angle", angle);
    if (!finished2 && Math.abs(dist - 5.9)>0.4) {
      tankDrive(-Math.signum(dist-5.9)*0.23,-Math.signum(dist-5.9)* 0.23);
    }
    else{
     tankDrive(0,0);
      b = new LinkedList<Double>();
      finished2 = true;
  }
  // float distance = m_hubVision.getZ();
  // speed = calcTrajectory.calcSpeed(distance, Constants.PhysicalConsts.SHOOT_HEIGHT,
  //  Constants.PhysicalConsts.SHOOT_ANGLE);
  //  m_shootPidMotor.set(speed * Constants.Speeds.SHOOT);
}
public static void unDriveStright(){
  finished2 = false;
  
}
  private static PIDController turnPID = new PIDController(Constants.PIDValues.TURN_KP,
   Constants.PIDValues.TURN_KI, Constants.PIDValues.TURN_KD);
   private static PIDController movePID = new PIDController(0.0001,
   0, 0);
  public static LinkedList<Double> a = new LinkedList<Double>();
  public static LinkedList<Double> b = new LinkedList<Double>();

  public static boolean finished = false;
  public static void turnToShoot(){
    
    double angle = m_hubVision.getAngleX();
    a.add(angle);
    if(a.size() > 5){ 
    double myAng = a.getLast() + a.get(a.size()-2) + a.get(a.size() -3);
    angle = myAng/3;
    }

    // double speed = turnPID.calculate(angle,0);
    //SmartDashboard.putNumber(*"turnSpeed", speed);
    //SmartDashboard.putNumber("angle", angle);
    if (!finished && Math.abs(angle)>3) {
      tankDrive(-Math.signum(angle)*0.17, Math.signum(angle)*0.17);
    }
    else{
      tankDrive(0,0);
      a = new LinkedList<Double>();
      finished = true;
    }
    // float distance = m_hubVision.getZ();
    // speed = calcTrajectory.calcSpeed(distance, Constants.PhysicalConsts.SHOOT_HEIGHT,
    //  Constants.PhysicalConsts.SHOOT_ANGLE);
    //  m_shootPidMotor.set(speed * Constants.Speeds.SHOOT);
  }
  public static void unTurnToShoot(){
    turnPID.reset();
    finished = false;
  }
  
  public static void shoot() {
    float distance = m_hubVision.getZ();
    double target = calcTrajectory.calcSpeed(distance, Constants.PhysicalConsts.SHOOT_HEIGHT,
    Constants.PhysicalConsts.SHOOT_ANGLE) * Constants.Speeds.SHOOT;
    m_shootPidMotor.setTarget(23);
    SmartDashboard.putNumber("target", target);
    m_shootPidMotor.runMotor();
  }

  public static void shootAuto() {
    float distance = m_hubVision.getZ();
    // double speed = calcTrajectory.calcSpeed(distance, Constants.PhysicalConsts.SHOOT_HEIGHT,
    // Constants.PhysicalConsts.SHOOT_ANGLE);
    // m_shootPidMotor.set(speed * Constants.Speeds.SHOOT);
    // m_shootPidMotor.runMotor();
    Constants.UsableMotors.FLY_WHEEL.set(ControlMode.PercentOutput, 0.69 * 0.87);
  }
  
  

  public static void movePeriodic() {

    // init();
    // Move
    driveByJoysticks();

    // collector open
    MotorController collectorOpen = Constants.UsableMotors.COLLECTOR_OPEN;
    //SmartDashboard.putBoolean("y button:", m_collectorOpen.get());
    double speed = ((m_collectorOpen.get() ? 1 : 0) - (m_collectorClose.get() ? 1 : 0))
     * Constants.Speeds.collectorOpen;
    collectorOpen.set(speed);

    // collector collect
    MotorController collectorCollect = Constants.UsableMotors.COLLECTOR_COLLECT;
    speed = ((m_collectorCollect.get() ? 1 : 0) - (m_collectorUncollect.get() ? 1 : 0))
    * Constants.Speeds.collectorCollect;
    collectorCollect.set(speed);

    // storage
    MotorController bottomStorage = Constants.UsableMotors.BOTTOM_STORAGE;
    speed = ((m_storageUp.get() ? 1 : 0) - (m_storageDown.get() ? 1 : 0)) * Constants.Speeds.storageMotor;
    bottomStorage.set(speed);
    
    
    // shoot
    if (m_turnToShoot.get()){
      turnToShoot();
    }else{
      unTurnToShoot();
    }
    if (m_load.get()){
      Constants.UsableMotors.STORAGE_TOP.set(Constants.Speeds.storageMotor);
    }
    else{
      Constants.UsableMotors.STORAGE_TOP.set(0);
    }
    //SmartDashboard.putBoolean("shoot: ", m_fire.get());
    if (m_fire.get()){
      shoot();
    }
    else{
      m_shootPidMotor.stop();
      Constants.UsableMotors.STORAGE_TOP.set(Constants.UsableMotors.STORAGE_TOP.get() +
        (m_storageDown.get() ? -1 : 0) * Constants.Speeds.storageMotor);
        Constants.PIDValues.FLY_WHEEL_KP = 0.00001;
    }
    if(m_driveStright.get()){
      driveStright();
    }else{
      unDriveStright();
    }

    
    
    // Climb
    Constants.UsableMotors.CLIMBER_MOTOR.set(((m_climberUp.get() ? 1 : 0) + (m_climberDown.get() ? -1 : 0)) * Constants.Speeds.climberMotor);
  
  }
}
