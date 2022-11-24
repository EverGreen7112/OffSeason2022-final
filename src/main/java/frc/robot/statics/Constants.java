package frc.robot.statics;
// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import frc.robot.EverLibEssentials.Motor;
import frc.robot.EverLibEssentials.MotorGroup;
import frc.robot.EverLibEssentials.Motors;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class MotorPorts {
        public static final int chassisRightFront = 12,
                chassisRightMiddle = 13,
                chassisRightBack = 3,

                chassisLeftFront = 1,
                chassisLeftMiddle = 15,
                chassisLeftBack = 14,

                collectorOpen = 7,
                collectorCollect = 5,

                bottomStorage = 2,
                topStorage = 6,
                flyWheel = 0,

                climber = 4;
    }

    public static class UsableMotors {
        // declare your motors here

        public static final Motor<WPI_VictorSPX> CLIMBER_MOTOR = new Motor<WPI_VictorSPX>(MotorPorts.climber,
                new WPI_VictorSPX(MotorPorts.climber));

        public static final MotorGroup CHASSIS_RIGHT = new MotorGroup(
                new Motor<WPI_VictorSPX>(MotorPorts.chassisRightFront, new WPI_VictorSPX(MotorPorts.chassisRightFront)),
                new Motor<WPI_VictorSPX>(MotorPorts.chassisRightMiddle,
                        new WPI_VictorSPX(MotorPorts.chassisRightMiddle)),
                new Motor<WPI_VictorSPX>(MotorPorts.chassisRightBack, new WPI_VictorSPX(MotorPorts.chassisRightBack)));

        public static final MotorGroup CHASSIS_LEFT = new MotorGroup(
                new Motor<WPI_VictorSPX>(MotorPorts.chassisLeftFront, new WPI_VictorSPX(MotorPorts.chassisLeftFront)),
                new Motor<WPI_VictorSPX>(MotorPorts.chassisLeftMiddle, new WPI_VictorSPX(MotorPorts.chassisLeftMiddle)),
                new Motor<WPI_VictorSPX>(MotorPorts.chassisLeftBack, new WPI_VictorSPX(MotorPorts.chassisLeftBack)));

        public static final Motor<WPI_VictorSPX> COLLECTOR_OPEN = new Motor<WPI_VictorSPX>(MotorPorts.collectorOpen,
                new WPI_VictorSPX(MotorPorts.collectorOpen));
        public static final Motor<WPI_VictorSPX> COLLECTOR_COLLECT = new Motor<WPI_VictorSPX>(
                MotorPorts.collectorCollect, new WPI_VictorSPX(MotorPorts.collectorCollect));
        public static final Motor<WPI_VictorSPX> BOTTOM_STORAGE = new Motor<WPI_VictorSPX>(MotorPorts.bottomStorage,
                new WPI_VictorSPX(MotorPorts.bottomStorage));
        public static final Motor<WPI_VictorSPX> STORAGE_TOP = new Motor<WPI_VictorSPX>(MotorPorts.topStorage,
                new WPI_VictorSPX(MotorPorts.topStorage));
        public static WPI_TalonSRX FLY_WHEEL = new WPI_TalonSRX(MotorPorts.flyWheel);

        // public static MotorGroup
        // CHASSIS_LEFT,
        // CHASSIS_RIGHT;
        public static void init() {
            // initialize your motors here

            // WPI_VictorSPX
            // leftFrontMotor = new WPI_VictorSPX(MotorPorts.chassisLeftFront),
            // rightFrontMotor = new WPI_VictorSPX(MotorPorts.chassisRightFront);
            // leftFrontMotor.setInverted(true);
            // rightFrontMotor.setInverted(true);

            // CHASSIS_LEFT = new MotorGroup(leftFrontMotor,
            // new WPI_VictorSPX[]{new WPI_VictorSPX(MotorPorts.chassisLeftMiddle),
            // new WPI_VictorSPX(MotorPorts.chassisLeftBack)});
            // CHASSIS_RIGHT = new MotorGroup(rightFrontMotor,
            // new WPI_VictorSPX[]{new WPI_VictorSPX(MotorPorts.chassisRightMiddle),
            // new WPI_VictorSPX(MotorPorts.chassisRightBack)});
            // CHASSIS_LEFT.set(0);
            // CHASSIS_RIGHT.set(0);

        }
    }

    public static class JoystickPorts {
        public static final int 
                rightJoystick = 0,
                leftJoystick = 1,
                operator = 2;
    }

    public static class ButtonPorts {
        public static final int
            // missing ports
            collectorOpen = 4,
            collectorClose = 2,
            collectorCollect = 1,
            collectorUncollect = 3,
            
            load = 8, 
        //     fire = 8, 
            turbo = 1,
            climberDown = 7,
            climberUp = 9,
            
            storageUp = 6,
            storageDown = 3,

        
            turnToShoot = 3; // right drive joystick, not operator
    }

    public static class Speeds {
        public static final double speedLimit = 0.8,
                moveSpeed = 0.6,
                collectorClose = -0.7,
                collectorOpen = 0.5,
                collectorCollect = 0.55,

                // SHOOT = 2.03, 
                climberMotor = 0.6,

                storageMotor = 0.5;
    }

    public static class ComPorts {
        public static final int hubVision = 5800,
         ballVision = 5801;

    }

    public static class PhysicalConsts {
        public static final double SHOOT_HEIGHT = 0.87,
                SHOOT_ANGLE = 84.6;// 60
    }


    public static class PIDValues {
        public static double 
                FLY_WHEEL_KP = 0.000008,               //0.00001 used to work //0.0000048
                FLY_WHEEL_KI = 0.00000000000000184,    //0.0000000000000013 used to work
                FLY_WHEEL_KD = 0.000005,                //0.000007 used to work


                TURN_KP = 0.007,
                TURN_KI = 0.0035,
                TURN_KD = 0.002;
        }
}
