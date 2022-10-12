package frc.robot.EverLibEssentials;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class MotorGroup extends MotorControllerGroup {

    public MotorGroup(Motor<?>... motorControllers) {
        super(motorControllers);
    }

}
