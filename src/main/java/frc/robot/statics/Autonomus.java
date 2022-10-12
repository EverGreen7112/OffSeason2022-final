package frc.robot.statics;

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
}
