package frc.robot.statics;

import frc.robot.EverLibEssentials.Motors;

public class Autonomus {
    private static Long startTime;
    private static int eDrive = 3000;
    private static int eLoad = 8000;
    
    public static void init(){
        startTime = System.currentTimeMillis();
    }

    public static void emergencyAuto(){
        long curTime = System.currentTimeMillis();
        if (startTime + eDrive < curTime){
            Controls.tankDrive(Constants.Speeds.moveSpeed, Constants.Speeds.moveSpeed);
        }
        else if (startTime + eLoad < curTime ){
            Controls.tankDrive(0, 0);
            Controls.turnToShoot();
            Controls.shoot();
        }
        else{
            Constants.UsableMotors.BOTTOM_STORAGE.set(Constants.Speeds.storageMotor);
        }
    }
    public static void auto(){
        startTime = System.currentTimeMillis();

        Controls.tankDrive(0.3, 0.3);
        Motors.runMotors();

        while(System.currentTimeMillis() - startTime < 1.7*1000);
        Controls.tankDrive(0, 0);
        Motors.runMotors();

        Controls.unDriveStright();
        Controls.shootAuto();
        Motors.runMotors();

        while(System.currentTimeMillis() - startTime < 6.7*1000);
        Constants.UsableMotors.STORAGE_TOP.set(Constants.Speeds.storageMotor);
        Motors.runMotors();
        while(System.currentTimeMillis() - startTime < 9*1000);

        Constants.UsableMotors.BOTTOM_STORAGE.set(Constants.Speeds.collectorCollect);
        Motors.runMotors();


    }
}
