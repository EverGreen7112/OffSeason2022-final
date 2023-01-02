package frc.robot.statics;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.EverLibEssentials.Motor;

public class PIDMotor {
    private TalonSRX m_motor;
    private double m_kp, m_ki, m_kd;
    private double m_lastError = 0, m_integral = 0, m_target = 0, m_setSpeed; //m_setSpeed is the precentage [-1 to 1]
    private Thread m_PIDThread;

    public PIDMotor(TalonSRX motor, double kp, double ki, double kd){
        this.m_motor = motor;
        this.m_kp = kp;
        this.m_ki = ki;
        this.m_kd = kd;
        this.m_motor.configFactoryDefault();
        
        m_PIDThread = new Thread(()->{
            while(true){
                double speed = this.m_motor.getSelectedSensorVelocity() / -Constants.Values.TICKS_PER_REVOLUTIONS; //make 100ms to minute
                // PID               
                SmartDashboard.putNumber("speed", speed);
                SmartDashboard.putNumber("vel", m_motor.getSelectedSensorVelocity());
                SmartDashboard.putNumber("distance", m_motor.getSelectedSensorPosition());
                double error = this.m_target - speed;
                double finalSpeed = error * this.m_kp; // P
                finalSpeed += this.m_integral * this.m_ki; // I
                finalSpeed -= (error - this.m_lastError) * this.m_kd; // D
                this.m_setSpeed += finalSpeed;
                //runMotor();
            }
        });
        m_PIDThread.setDaemon(true);
        m_PIDThread.start();
    }
    

    public void setTarget(double target) {
        this.m_target = target;
    }

    public void runMotor(){
        if(m_setSpeed < 0){
            m_setSpeed = 0;
            return;
        }
        SmartDashboard.putNumber("set speed", m_setSpeed);
        SmartDashboard.putNumber("actual", Math.min(Math.max(m_setSpeed, -1),1));
        this.m_motor.set(ControlMode.PercentOutput, Math.min(Math.max(m_setSpeed, -1),1));
    }

    public void stop(){
        this.m_motor.set(ControlMode.PercentOutput, 0);
    }
    public void reset(){
        this.m_setSpeed = 0;
        this.m_integral = 0;
        this.m_target = 0;
        this.m_lastError = 0;
    }
}