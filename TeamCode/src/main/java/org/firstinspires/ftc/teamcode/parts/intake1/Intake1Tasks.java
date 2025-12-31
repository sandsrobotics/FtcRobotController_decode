package org.firstinspires.ftc.teamcode.parts.intake1;

import om.self.ezftc.core.Robot;
import om.self.task.core.Group;
import om.self.task.other.TimedTask;

public class Intake1Tasks {
    public final Group intakeTasksGroup;
    public final TimedTask intakeTask;
    public final TimedTask artifactIntakeStopTask;
    public final TimedTask outtakeTask;
    public final TimedTask pinkServoTransfer;
    public final TimedTask blueServoTransfer;
    public final TimedTask greenServoTransfer;
    public final TimedTask allServoTransfer;
    public final TimedTask startFarLaunch;
    public final TimedTask startGoalLaunch;
    public final TimedTask stopLaunch;

    private final Intake1 intake;
    private final Robot robot;


    public Intake1Tasks(Intake1 intake, Robot robot) {
        this.intake = intake;
        this.robot = robot;
        intakeTasksGroup = new Group("intake", intake.getTaskManager());
        intakeTask = new TimedTask(TaskNames.intakeTask, intakeTasksGroup);
        artifactIntakeStopTask = new TimedTask(TaskNames.artifactIntakeStop, intakeTasksGroup);
        outtakeTask = new TimedTask(TaskNames.outtakeTask, intakeTasksGroup);
        pinkServoTransfer = new TimedTask(TaskNames.pinkServoTransfer, intakeTasksGroup);
        blueServoTransfer = new TimedTask(TaskNames.blueServoTransfer, intakeTasksGroup);
        greenServoTransfer = new TimedTask(TaskNames.greenServoTransfer, intakeTasksGroup);
        allServoTransfer = new TimedTask(TaskNames.allServoTransfer, intakeTasksGroup);
        startFarLaunch = new TimedTask(TaskNames.startFarLaunch, intakeTasksGroup);
        startGoalLaunch = new TimedTask(TaskNames.startGoalLaunch, intakeTasksGroup);
        stopLaunch = new TimedTask(TaskNames.stopLaunch, intakeTasksGroup);
    }

    public void constructAllIntakeTasks() {
        /*     Motor Intake Task  */
        intakeTask.autoStart = false;
        intakeTask.addStep(()->{
            intake.getHardware().intakeMotor.setPower(intake.getSettings().intakeIn);
        });

        /*   Artifact Intake Stop Task   */
        artifactIntakeStopTask.autoStart = false;
        artifactIntakeStopTask.addStep(()-> {
            intake.getHardware().intakeMotor.setPower(intake.getSettings().intakeStop);
        });

        /*    Artifact Extake Task*/
        outtakeTask.autoStart = false;
        outtakeTask.addStep(() -> intake.getHardware().intakeMotor.setPower(intake.getSettings().intakeOut));

        /*    Servo Transfer Tasks      */
        //         pink
        pinkServoTransfer.autoStart = false;
        pinkServoTransfer.addStep(() -> intake.getHardware().pinkServo.setPosition(intake.getSettings().servoPinkLaunch));
        pinkServoTransfer.addStep(() -> intake.getHardware().pinkServo.isDone());
        pinkServoTransfer.addDelay(250);
        pinkServoTransfer.addStep(() -> intake.getHardware().pinkServo.setPosition(intake.getSettings().servoPinkDock));
        //          blue
        blueServoTransfer.autoStart = false;
        blueServoTransfer.addStep(() -> intake.getHardware().blueServo.setPosition(intake.getSettings().servoBlueLaunch));
        blueServoTransfer.addStep(() -> intake.getHardware().blueServo.isDone());
        blueServoTransfer.addDelay(250);
        blueServoTransfer.addStep(() -> intake.getHardware().blueServo.setPosition(intake.getSettings().servoBlueDock));
        //        green
        greenServoTransfer.autoStart = false;
        greenServoTransfer.addStep(() -> intake.getHardware().greenServo.setPosition(intake.getSettings().servoGreenLaunch));
        greenServoTransfer.addStep(() -> intake.getHardware().greenServo.isDone());
        greenServoTransfer.addDelay(250);
        greenServoTransfer.addStep(() -> intake.getHardware().greenServo.setPosition(intake.getSettings().servoGreenDock));
        //           all
        allServoTransfer.autoStart = false;
        allServoTransfer.addStep(()->{
            pinkServoTransfer.restart();
            blueServoTransfer.restart();
            greenServoTransfer.restart();
        });
        allServoTransfer.addStep(() ->
            intake.getHardware().greenServo.isDone() && intake.getHardware().blueServo.isDone() && intake.getHardware().pinkServo.isDone()
        );
        /*    Launch Tasks         */
        //    start Launch
        startFarLaunch.autoStart = false;
        startFarLaunch.addStep(() -> {
            intake.getHardware().launchMotorLeft.setVelocity(intake.getSettings().farLaunchMotorVelocityStart);
            intake.getHardware().launchMotorRight.setVelocity(intake.getSettings().farLaunchMotorVelocityStart);
        });

        startGoalLaunch.autoStart = false;
        startGoalLaunch.addStep(() -> {
            intake.getHardware().launchMotorLeft.setVelocity(intake.getSettings().goalLaunchMotorVelocityStart);
            intake.getHardware().launchMotorRight.setVelocity(intake.getSettings().goalLaunchMotorVelocityStart);
        });
        //     stop launch
        stopLaunch.autoStart = false;
        stopLaunch.addStep(() -> {
            intake.getHardware().launchMotorLeft.setVelocity(intake.getSettings().launchMotorVelocityStop);
            intake.getHardware().launchMotorRight.setVelocity(intake.getSettings().launchMotorVelocityStop);
        });

    }
    /***********************************************************************************/
    public static final class TaskNames {
        public final static String intakeTask = "motor intake";
        public final static String artifactIntakeStop = "artifact intake stop";
        public final static String outtakeTask = "outtake Task";
        public final static String pinkServoTransfer = "pink servo transfer";
        public final static String blueServoTransfer = "blue servo transfer";
        public final static String greenServoTransfer = "green servo transfer";
        public final static String allServoTransfer = "all servos transfer";
        public final static String startFarLaunch = "start FarLaunch";
        public final static String stopLaunch = "stop Launch";
        public final static String startGoalLaunch = "start GoalLaunch";
    }

    public static final class Events {
        public static  final String homeComplete = "HOME_COMPLETE";
    }
}
