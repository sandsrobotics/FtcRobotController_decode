package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.parts.intake3.settings.IntakeSettings3;
import org.firstinspires.ftc.teamcode.parts.intake3.settings.IntakeSettings3.*;
import org.firstinspires.ftc.teamcode.parts.positionsolver.settings.PositionSolverSettings;

import om.self.ezftc.utils.Vector3;
import om.self.task.other.TimedTask;

@Autonomous(name="Blue Goal", group="32859")
public class T3_AutoBlueGoal extends T3_AutoBase {
    Vector3 blueLaunchStart = new Vector3(-40,-55,180);; //-58.8,-45,140

    // CONFIGURABLE PICKUP TIMEOUT (in milliseconds)
    // Adjust this value to control how long the robot tries to pickup balls on each spike
    private final int SPIKE_PICKUP_TIMEOUT = 2500; // 5 second default, change as needed



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
        int localSpikeCount = runSpikeCount; // Get from base class


        Vector3 start = fieldStartPos; //new Vector3(-51, -50, 140);
        Vector3 aprilTag = transformFunc.apply(new Vector3(-42, -41, 140)); //z=122

        LaunchData shootLaunchData = new LaunchData(2350, transformFunc.apply(new Vector3(-26, -14, -128 )));

        Vector3 blueSpikeReady1 = transformFunc.apply(new Vector3(-12,-28,-90));
        Vector3 blueSpike1 = transformFunc.apply(new Vector3(-12,-53,-90));
        Vector3 blueSpikeReady2 = transformFunc.apply(new Vector3(13,-24,-90));
        Vector3 blueSpike2 = transformFunc.apply(new Vector3(14,-60,-90));
        Vector3 blueSpikeReady3 = transformFunc.apply(new Vector3(35.5,-20,-90)); //x was 35, needed to go left more
        Vector3 blueSpike3 = transformFunc.apply(new Vector3(35.5,-60,-90)); //x was 35, needed to go left more

        /* *** setup Autonomous ****/
        autoTasks.addStep(() -> intake.stopAllIntakeTasks());
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultSettings));
        autoTasks.addStep(() -> odo.setPosition(start));

        // start launcher before first move to get it up to speed before timing out
        autoTasks.addStep(() -> intake.setLaunchRPM(shootLaunchData.getRPM()));
        positionSolver.addMoveToTaskEx(aprilTag, autoTasks);
        MoveAndLaunch(autoTasks, shootLaunchData, false);

        // SPIKE 1 - with timeout
        if(runSpikeCount >= 1) {
            autoTasks.addStep(() -> intake.setIntakeRPM(IntakeSettings3.intakeRPM));
            positionSolver.addMoveToTaskEx(blueSpikeReady1, autoTasks);
            autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.superSlowSettings));
            positionSolver.addMoveToTaskEx(blueSpike1, autoTasks, SPIKE_PICKUP_TIMEOUT);
            autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultSettings));
            MoveAndLaunch(autoTasks, shootLaunchData);
        }

        // SPIKE 2 - with timeout
        if(runSpikeCount >= 2) {
            autoTasks.addStep(() -> intake.setIntakeRPM(IntakeSettings3.intakeRPM));
            positionSolver.addMoveToTaskEx(blueSpikeReady2, autoTasks);
            autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.superSlowSettings));
            positionSolver.addMoveToTaskEx(blueSpike2, autoTasks, SPIKE_PICKUP_TIMEOUT);
            autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));
            positionSolver.addMoveToTaskEx(blueSpikeReady2, autoTasks);
            autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultSettings));
            MoveAndLaunch(autoTasks, shootLaunchData);
        }

        // SPIKE 3 - with timeout
//        if(runSpikeCount >= 3) {
//            autoTasks.addStep(() -> intake.setIntakeRPM(IntakeSettings3.intakeRPM));
//            positionSolver.addMoveToTaskEx(blueSpikeReady3, autoTasks);
//            autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.ultraSlowSettings));
//            positionSolver.addMoveToTaskEx(blueSpike3, autoTasks, SPIKE_PICKUP_TIMEOUT);
//            autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultSettings));
//            MoveAndLaunch(autoTasks, shootLaunchData);
//        }

        positionSolver.addMoveToTaskEx(blueSpike1, autoTasks); //Goes off launch line
        autoTasks.addStep(() -> intake.setIntakeRPM(0));
    }

    /************************************************************/
    private void MoveAndLaunch(TimedTask autoTasks, LaunchData launchData) {
        MoveAndLaunch(autoTasks, launchData, true);
    }
    private void MoveAndLaunch(TimedTask autoTasks, LaunchData launchData, Boolean rejectExtraArtifacts) {
        int RPM = launchData.getRPM();
        Vector3 launchPos = launchData.getPosition();
        if(rejectExtraArtifacts) {
            autoTasks.addDelay(600); // tjk to let balls all get in
        }
        positionSolver.addMoveToTaskExNoWait(launchPos, autoTasks);
        //autoTasks.addStep(() -> intake.setLaunchRPM(RPM));

        // Reject extra artifacts
        if (rejectExtraArtifacts) {
            autoTasks.addTimedStep(
                    () -> intake.setIntakeRPM(-IntakeSettings3.intakeRPM),
                    () -> positionSolver.isDone(),
                    1500
            );
        }

        autoTasks.addStep(() -> intake.setIntakeRPM(0));

        // Use ordered color launch instead of regular ball launch
//        autoTasks.addStep(intake.tasks.sameTimeBallLaunchTask::restart);
//        autoTasks.addStep(intake.tasks.sameTimeBallLaunchTask::isDone);
        autoTasks.addStep(intake.tasks.orderedColorLaunchTask::restart);
        autoTasks.addStep(intake.tasks.orderedColorLaunchTask::isDone);
    }
}