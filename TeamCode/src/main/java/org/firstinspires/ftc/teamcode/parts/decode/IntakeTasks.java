package org.firstinspires.ftc.teamcode.parts.decode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.parts.decode.settings.IntakeSettings;

import om.self.ezftc.core.Robot;
import om.self.task.core.Group;
import om.self.task.other.TimedTask;

public class IntakeTasks {
    public final Group intakeTasksGroup;
    public final TimedTask artifactIntakeTask;
    public final TimedTask artifactExtakeTask;
    public final TimedTask artifactIntakeStopTask;

    private final Intake intake;
    private final Robot robot;



    public IntakeTasks(Intake intake, Robot robot) {
        this.intake = intake;
        this.robot = robot;
        intakeTasksGroup = new Group("intake", intake.getTaskManager());
        artifactIntakeTask = new TimedTask(TaskNames.artifactIntakeTask, intakeTasksGroup);
        artifactExtakeTask = new TimedTask(TaskNames.artifactExtakeTask, intakeTasksGroup);
        artifactIntakeStopTask = new TimedTask(TaskNames.artifactIntakeStop, intakeTasksGroup);


    }

    public void constructAllIntakeTasks() {
        /*     Artifact Intake Task  */
        artifactIntakeTask.autoStart = false;
        artifactIntakeTask.addStep(()->{
            intake.getHardware().intakeMotor.setPower(intake.getSettings().motorIntakeIn);
            intake.setServoIntake(intake.getSettings().servoIntakeIn);
        });
        /*    Artifact Extake Task */
        artifactExtakeTask.autoStart = false;
        artifactExtakeTask.addStep(()->{
            intake.getHardware().intakeMotor.setPower(intake.getSettings().motorIntakeOut);
            intake.setServoIntake(intake.getSettings().servoIntakeOut);
            intake.getHardware().intakeServo.setPosition(intake.getSettings().servoIntakeOut);
        });
        /*   Artifact Intake Stop Task   */
        artifactIntakeStopTask.autoStart = false;
        artifactIntakeStopTask.addStep(()-> {
            intake.getHardware().intakeMotor.setPower(intake.getSettings().motorIntakeStop);
            intake.setServoIntake(intake.getSettings().servoIntakeStop);
        });






    }
    /***********************************************************************************/
    public static final class TaskNames {
        public final static String artifactIntakeTask = "artifact intake";
        public final static String artifactExtakeTask = "artifact extake";
        public final static String artifactIntakeStop = "artifact intake stop";

    }

    public static final class Events {
        public static  final String homeComplete = "HOME_COMPLETE";
    }
}
