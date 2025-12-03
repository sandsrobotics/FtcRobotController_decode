package org.firstinspires.ftc.teamcode.parts.intake3;

//import org.firstinspires.ftc.teamcode.parts.intake3.settings.IntakeSettings3;

import om.self.ezftc.core.Robot;
import om.self.task.core.Group;
import om.self.task.other.TimedTask;

public class Intake3Tasks {
    protected final Group movementTask;
    private final TimedTask homeTask;
    public final TimedTask ballLaunchTask;
    private final Intake3 intake;
    private final Robot robot;

    public Intake3Tasks(Intake3 intake, Robot robot) {
        this.intake = intake;
        this.robot = robot;
        movementTask = new Group("auto movement", intake.getTaskManager());
        homeTask = new TimedTask(TaskNames.Home, movementTask);
        ballLaunchTask= new TimedTask(TaskNames.BallLaunch, movementTask);
    }

    public void constructAllIntakeTasks() {
        homeTask.autoStart = false;
        /* ***** autoBallLaunchTask ******/
        ballLaunchTask.autoStart = false;
        ballLaunchTask.addStep(()-> intake.getHardware().pixel.setPosition(Intake3.LEDColor.BLUE.getLedPwm()));
        ballLaunchTask.addStep(()-> intake.getHardware().launchServo.setPosition(.9));
        ballLaunchTask.addStep(()-> intake.getHardware().launchServo.isDone());
        ballLaunchTask.addDelay(1000);
        ballLaunchTask.addStep(()-> intake.getHardware().launchServo.setPosition(0));
    }

    /***********************************************************************************/
    public static final class TaskNames {
        public final static String Home = "auto home";
        public final static String BallLaunch = "auto ball launch";
    }

    public static final class Events {
        public static  final String homeComplete = "HOME_COMPLETE";
    }
}
