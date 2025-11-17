package org.firstinspires.ftc.teamcode.parts.intake3;

import om.self.ezftc.core.Robot;
import om.self.task.core.Group;
import om.self.task.other.TimedTask;

public class Intake3Tasks {
    protected final Group movementTask;
    private final TimedTask autoHomeTask;
    public final TimedTask autoBallLaunchTask;
    private Intake3 intake;
    private final Robot robot;

    public Intake3Tasks(Intake3 intake, Robot robot) {
        this.intake = intake;
        this.robot = robot;
        movementTask = new Group("auto movement", intake.getTaskManager());
        autoHomeTask = new TimedTask(TaskNames.autoHome, movementTask);
        autoBallLaunchTask= new TimedTask(TaskNames.autoBallLaunch, movementTask);
    }

    public void constructAllIntakeTasks() {

        /* ***** autoHomeTask ******/
        autoHomeTask.autoStart = false;
//        autoHomeTask.addStep(this::setSlideToHomeConfig);
//        autoHomeTask.addStep(() -> {
//            intake.incrementIntakeUpDown(0);
//            intake.getHardware().specimenServo.setPosition(intake.getSettings().specimenServoOpenPosition);
//            intake.getHardware().dropperServo.setPosition(intake.getSettings().dropperDockSafe);
//            intake.getHardware().dropperServo.isDone();
//            intake.getHardware().dropperServo.disable();
//        });
//        autoHomeTask.addTimedStep(() -> {
//            robot.opMode.telemetry.addData("homing", intake.getHardware().bucketLiftZeroSwitch.getState());
//        }, () -> intake.getHardware().bucketLiftZeroSwitch.getState(), 10000);
//        autoHomeTask.addStep(() -> {
//            intake.getHardware().bucketLiftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//            intake.getHardware().bucketLiftMotor.setTargetPosition(20);
//            intake.bucketLiftTargetPosition = 20;
//            setMotorsToRunConfig();
//        });

        /* ***** autoBucketLiftTask ******/
        autoBallLaunchTask.autoStart = false;
//        autoBucketLiftTask.autoStart = false;
////        autoBucketLiftTask.addStep(()-> bucketliftinprogress = true);
//        autoBucketLiftTask.addStep(() -> intake.getHardware().tiltServo.setPosition(intake.getSettings().intakeArmSafe));
//        autoBucketLiftTask.addStep(() -> intake.getHardware().tiltServo.isDone());
//        //Todo: maybe drive droper down if needed
//        // autoBucketLiftTask.addStep(()-> intake.getHardware().dropperServo.setPosition(intake.getSettings().dropperDockSafe);
//        autoBucketLiftTask.addStep(() -> intake.getHardware().dropperServo.disable());
//        autoBucketLiftTask.addStep(() -> intake.setLiftPosition(intake.getSettings().maxLiftPosition, 1));
//        autoBucketLiftTask.addStep(() -> intake.getHardware().bucketLiftMotor.getCurrentPosition() > 1100);
//        autoBucketLiftTask.addStep(() -> intake.getHardware().dropperServo.enable());
//        autoBucketLiftTask.addStep(() -> intake.getHardware().dropperServo.setPosition(intake.getSettings().dropperServoMin));
//        autoBucketLiftTask.addStep(() -> intake.getHardware().bucketLiftMotor.getCurrentPosition() > 2600);
//        autoBucketLiftTask.addStep(()-> bucketliftinprogress = false);
//        autoBucketLiftTask.addStep(()-> intake.getHardware().dropperServo.isDone()); //if greater than 500 set servo straight
    }

    /***********************************************************************************/
    public static final class TaskNames {
        public final static String autoHome = "auto home";
        public final static String autoBallLaunch = "auto ball launch";
    }

    public static final class Events {
        public static  final String homeComplete = "HOME_COMPLETE";
    }
}
