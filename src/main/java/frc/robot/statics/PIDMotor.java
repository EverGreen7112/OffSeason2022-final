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
    // private int m_full = 10000;

    public PIDMotor(TalonSRX motor, double kp, double ki, double kd){
        this.m_motor = motor;
        this.m_kp = kp;
        this.m_ki = ki;
        this.m_kd = kd;
        this.m_motor.configFactoryDefault();

        m_PIDThread = new Thread(()->{
            // double lastDistance = m_encoder.getDistance();
            // int lastTime = (int)(System.currentTimeMillis() % this.m_full);
            while(true){
                // double distance = m_encoder.getDistance();
                // int time = (int)(System.currentTimeMillis() % this.m_full);
                // double speed;
                // try {
                //     speed = (distance - lastDistance) / (time - lastTime);
                // } catch (Exception e) {
                //     speed = 0;
                // }
                double speed = -this.m_motor.getSelectedSensorVelocity() * Math.PI/8230;
                // PID
                SmartDashboard.putNumber("speed", speed);
                double error = this.m_target - speed;
                double finalSpeed = error * this.m_kp; // P
                finalSpeed += this.m_integral * this.m_ki; // I
                finalSpeed -= (error - this.m_lastError) *this.m_kd; // D
                //SmartDashboard.putNumber("finalSpeed", finalSpeed);
                //SmartDashboard.putNumber("setSpeed", m_setSpeed);

                this.m_setSpeed += finalSpeed;
                SmartDashboard.putNumber("set speed", m_setSpeed);
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
        if(-Constants.UsableMotors.FLY_WHEEL.getSelectedSensorVelocity() * Math.PI/8230 >= m_target){
            this.m_motor.set(ControlMode.PercentOutput, 0.1);
            // Constants.PIDValues.FLY_WHEEL_KP = 0.000000000000000000000001;
        }
        else{
            // this.m_motor.set(ControlMode.PercentOutput, Math.min(Math.max(m_setSpeed, -1),1));
        }
        // this.m_motor.set(ControlMode.Velocity, 23);
    }

    public void stop(){
        m_lastError = 0;
        m_integral = 0;
        m_target = 0;
        this.m_motor.set(ControlMode.PercentOutput, 0);
    }
}
