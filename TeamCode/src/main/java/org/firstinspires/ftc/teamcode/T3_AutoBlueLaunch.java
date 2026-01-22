package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.parts.intake3.settings.IntakeSettings3;
import org.firstinspires.ftc.teamcode.parts.intake3.settings.IntakeSettings3.*;
import org.firstinspires.ftc.teamcode.parts.positionsolver.settings.PositionSolverSettings;

import om.self.ezftc.utils.Vector3;
import om.self.task.other.TimedTask;

@Autonomous(name="Blue Goal", group="32859")
public class T3_AutoBlueLaunch extends T3_AutoBase {
    Vector3 blueLaunchStart = new Vector3(-39.61,-54.79,180);; //-58.8,-45,140

    // CONFIGURABLE PICKUP TIMEOUT (in milliseconds)
    // Adjust this value to control how long the robot tries to pickup balls on each spike
    private final int SPIKE_PICKUP_TIMEOUT = 5000; // 5 second default, change as needed

    @Override
    public void initAuto(){
        isRedSide = false;
        transformFunc = (v) -> v;
        fieldStartPos = blueLaunchStart;
    }

    //        autoTasks.addDelay(1500);
    //        autoTasks.addStep(() -> intake.setIntakeRPM(-500)); // for reverse intake

    @Override
    public void BaseAuto(TimedTask autoTasks) {
        Vector3 start = fieldStartPos; //new Vector3(-51, -50, 140);
        Vector3 aprilTag = transformFunc.apply(new Vector3(-42, -41, 140)); //z=122

        LaunchData shootLaunchData = new LaunchData(2600, transformFunc.apply(new Vector3(-26.188, -14.106, -129.448)));

        Vector3 blueSpikeReady1 = transformFunc.apply(new Vector3(-12,-28,-90));
        Vector3 blueSpike1 = transformFunc.apply(new Vector3(-12,-53,-90));
        Vector3 blueSpikeReady2 = transformFunc.apply(new Vector3(13,-24,-90));
        Vector3 blueSpike2 = transformFunc.apply(new Vector3(13,-60,-90));
        Vector3 blueSpikeReady3 = transformFunc.apply(new Vector3(35.5,-20,-90)); //x was 35, needed to go left more
        Vector3 blueSpike3 = transformFunc.apply(new Vector3(35.5,-60,-90)); //x was 35, needed to go left more

        /* *** setup Autonomous ****/
        autoTasks.addStep(() -> intake.stopAllIntakeTasks());
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultSettings));
        autoTasks.addStep(() -> odo.setPosition(start));

        positionSolver.addMoveToTaskEx(aprilTag, autoTasks);

        MoveAndLaunch(autoTasks, shootLaunchData);

        // SPIKE 1 - with timeout
        autoTasks.addStep(() -> intake.setIntakeRPM(IntakeSettings3.intakeRPM));
        positionSolver.addMoveToTaskEx(blueSpikeReady1, autoTasks);
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.ultraSlowSettings));
        positionSolver.addMoveToTaskEx(blueSpike1, autoTasks, SPIKE_PICKUP_TIMEOUT);
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultSettings));
        MoveAndLaunch(autoTasks, shootLaunchData);

        // SPIKE 2 - with timeout
        autoTasks.addStep(() -> intake.setIntakeRPM(IntakeSettings3.intakeRPM));
        positionSolver.addMoveToTaskEx(blueSpikeReady2, autoTasks);
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.ultraSlowSettings));
        positionSolver.addMoveToTaskEx(blueSpike2, autoTasks, SPIKE_PICKUP_TIMEOUT);
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));
        positionSolver.addMoveToTaskEx(blueSpikeReady2, autoTasks);
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultSettings));
        MoveAndLaunch(autoTasks, shootLaunchData);

        // SPIKE 3 - with timeout
        autoTasks.addStep(() -> intake.setIntakeRPM(IntakeSettings3.intakeRPM));
        positionSolver.addMoveToTaskEx(blueSpikeReady3, autoTasks);
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.ultraSlowSettings));
        positionSolver.addMoveToTaskEx(blueSpike3, autoTasks, SPIKE_PICKUP_TIMEOUT);
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultSettings));
        MoveAndLaunch(autoTasks, shootLaunchData);

        positionSolver.addMoveToTaskEx(blueSpike1, autoTasks); //Goes off launch line
        autoTasks.addStep(() -> intake.setIntakeRPM(0));
        autoTasks.addStep(() -> intake.setLaunchRPM(0));
    }

    /************************************************************/
    private void MoveAndLaunch(TimedTask autoTasks, LaunchData launchData) {
        int RPM = launchData.getRPM();
        Vector3 pos = launchData.getPosition();
        positionSolver.addMoveToTaskExNoWait(pos, autoTasks);
        autoTasks.addStep(() -> intake.setLaunchRPM(RPM));

        // Feed artifacts in
        autoTasks.addTimedStep(
                () -> intake.setIntakeRPM(-500),
                () -> positionSolver.isDone(),
                3000
        );

        autoTasks.addStep(() -> intake.setIntakeRPM(0));

        // Use ordered color launch instead of regular ball launch
        autoTasks.addStep(intake.tasks.orderedColorLaunchTask::restart);
        autoTasks.addStep(intake.tasks.orderedColorLaunchTask::isDone);
    }
}