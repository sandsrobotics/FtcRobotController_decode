package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.parts.intake3.settings.IntakeSettings3;
import org.firstinspires.ftc.teamcode.parts.positionsolver.settings.PositionSolverSettings;
import om.self.ezftc.utils.Vector3;
import om.self.task.other.TimedTask;

@Autonomous(name="Blue TinyTriangle", group="32859")
public class T3_AutoBlueTinyTriangle extends T3_AutoBase {
    Vector3 blueLaunchStart = new Vector3(62.365, -15.197, 180);





    // CONFIGURABLE PICKUP TIMEOUT (in milliseconds)
    // Adjust this value to control how long the robot tries to pickup balls on each spike
    private final int SPIKE_PICKUP_TIMEOUT = 2500; // 5 second default, change as needed

    @Override
    public void initAuto(){
        isRedSide = false;
        transformFunc = (v) -> v;
        fieldStartPos = blueLaunchStart;
    }

    @Override
    public void BaseAuto(TimedTask autoTasks) {
        int localSpikeCount = runSpikeCount; // Get from base class


        Vector3 start = fieldStartPos;
        Vector3 aprilTag = transformFunc.apply(new Vector3(38.879, -24.293, 168));

        IntakeSettings3.LaunchData shootLaunchData = new IntakeSettings3.LaunchData(3275, transformFunc.apply(new Vector3(52.843, -13.030, -155.40 )));  /*Z: 153.4*/
        Vector3 blueSpikeReady1 = transformFunc.apply(new Vector3(-12,-28,-90));
        Vector3 blueSpike1 = transformFunc.apply(new Vector3(-12,-53,-90));
        Vector3 blueSpikeReady2 = transformFunc.apply(new Vector3(13,-28,-90));
        Vector3 blueSpike2 = transformFunc.apply(new  Vector3(13,-60,-90));
        Vector3 blueSpikeReady3 = transformFunc.apply(new Vector3(34.5,-28,-90));
        Vector3 blueSpike3 = transformFunc.apply(new Vector3(34,-60,-90));

        /* *** setup Autonomous ****/
        autoTasks.addStep(() -> intake.stopAllIntakeTasks());
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultSettings));
        autoTasks.addStep(() -> odo.setPosition(start));
        // start launcher before first move to get it up to speed before timing out
        autoTasks.addStep(() -> intake.setLaunchRPM(shootLaunchData.getRPM()));
//        positionSolver.addMoveToTaskEx(aprilTag, autoTasks); // can see april tag due to robot mods

        // Initial launch
        MoveAndLaunch(autoTasks, shootLaunchData, false);

        // Spike 3
//        SPIKE 1 - with timeout
        if(runSpikeCount >= 1) {
            autoTasks.addStep(() -> intake.setIntakeRPM(IntakeSettings3.intakeRPM));
            positionSolver.addMoveToTaskEx(blueSpikeReady3, autoTasks);
            autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.ultraSlowSettings));
            positionSolver.addMoveToTaskEx(blueSpike3, autoTasks, SPIKE_PICKUP_TIMEOUT);
            autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultSettings));
            autoTasks.addDelay(1000);
            autoTasks.addStep(() -> intake.setIntakeRPM(0));

            MoveAndLaunch(autoTasks, shootLaunchData);
        }

        // Spike 2
        if(runSpikeCount >= 2) {
            positionSolver.addMoveToTaskEx(blueSpikeReady2, autoTasks);
            autoTasks.addStep(() -> intake.setIntakeRPM(IntakeSettings3.intakeRPM));
            autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.ultraSlowSettings));
            positionSolver.addMoveToTaskEx(blueSpike2, autoTasks, SPIKE_PICKUP_TIMEOUT);
            autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultSettings));
            positionSolver.addMoveToTaskExNoWait(blueSpikeReady2, autoTasks);
            autoTasks.addDelay(1000);
            autoTasks.addStep(() -> intake.setIntakeRPM(0));

            MoveAndLaunch(autoTasks, shootLaunchData);
        }

        // Spike 1
        if(runSpikeCount >= 3) {
            autoTasks.addStep(() -> intake.setIntakeRPM(IntakeSettings3.intakeRPM));
            positionSolver.addMoveToTaskEx(blueSpikeReady1, autoTasks);
            autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.ultraSlowSettings));
            positionSolver.addMoveToTaskEx(blueSpike1, autoTasks, SPIKE_PICKUP_TIMEOUT);
            autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultSettings));
            autoTasks.addDelay(1000);
            autoTasks.addStep(() -> intake.setIntakeRPM(0));

            MoveAndLaunch(autoTasks, shootLaunchData);
        }

        autoTasks.addStep(() -> intake.setIntakeRPM(0));
    }

    /************************************************************/
    private void MoveAndLaunch(TimedTask autoTasks, IntakeSettings3.LaunchData launchData) {
        MoveAndLaunch(autoTasks, launchData, true);
    }
//    private void MoveAndLaunch(TimedTask autoTasks, IntakeSettings3.LaunchData launchData, Boolean rejectExtraArtifacts) {
//        int RPM = launchData.getRPM();
//        Vector3 pos = launchData.getPosition();
//        autoTasks.addDelay(1000); // tjk to let balls all get in
//        positionSolver.addMoveToTaskExNoWait(pos, autoTasks);
//        autoTasks.addStep(() -> intake.setLaunchRPM(RPM));
//
//        // Reject extra artifacts
//        if (rejectExtraArtifacts) {
//            autoTasks.addTimedStep(
//                    () -> intake.setIntakeRPM(-IntakeSettings3.intakeRPM),
//                    () -> positionSolver.isDone(),
//                    1500
//            );
//        }
//
//        autoTasks.addStep(() -> intake.setIntakeRPM(0));
//
//        // Use regular ball launch
//        autoTasks.addTimedStep(() -> {}, () -> intake.launchRPMInTolerance(), 3000);
//        autoTasks.addStep(intake.tasks.ballLaunchTask::restart);
//        autoTasks.addStep(intake.tasks.ballLaunchTask::isDone);
////        autoTasks.addDelay(1000);
////        autoTasks.addStep(() -> intake.setLaunchRPM(0));
//
//        // Use ordered color launch instead of regular ball launch
////        autoTasks.addStep(intake.tasks.orderedColorLaunchTask::restart);
////        autoTasks.addStep(intake.tasks.orderedColorLaunchTask::isDone);
//    }
    private void MoveAndLaunch(TimedTask autoTasks, IntakeSettings3.LaunchData launchData, Boolean rejectExtraArtifacts) {
        int RPM = launchData.getRPM();
        Vector3 launchPos = launchData.getPosition();
        autoTasks.addDelay(1000); // tjk to let balls all get in
        positionSolver.addMoveToTaskExNoWait(launchPos, autoTasks);
        autoTasks.addStep(() -> intake.setLaunchRPM(RPM));

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
        autoTasks.addStep(intake.tasks.orderedColorLaunchTask::restart);
        autoTasks.addStep(intake.tasks.orderedColorLaunchTask::isDone);
    }

}