package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.parts.intake3.settings.IntakeSettings3;
import org.firstinspires.ftc.teamcode.parts.positionsolver.settings.PositionSolverSettings;
import om.self.ezftc.utils.Vector3;
import om.self.task.other.TimedTask;

@Autonomous(name="Blue TinyTriangle", group="32859")
public class T3_AutoBlueTinyTriangle extends T3_AutoBase {
    Vector3 blueLaunchStart = new Vector3(62.365, -15.197, 180);

    // Flags to enable/disable going to certain spikes
    private boolean enableSpike1 = true;
    private boolean enableSpike2 = true;
    private boolean enableSpike3 = true;

    // CONFIGURABLE PICKUP TIMEOUT (in milliseconds)
    // Adjust this value to control how long the robot tries to pickup balls on each spike
    private final int SPIKE_PICKUP_TIMEOUT = 5000; // 5 second default, change as needed

    @Override
    public void initAuto(){
        isRedSide = false;
        transformFunc = (v) -> v;
        fieldStartPos = blueLaunchStart;
    }

    @Override
    public void BaseAuto(TimedTask autoTasks) {
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
        positionSolver.addMoveToTaskEx(aprilTag, autoTasks);

        // Initial launch
        MoveAndLaunch(autoTasks, shootLaunchData);

        // Spike 3
        if (enableSpike3) {
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
        if (enableSpike2) {
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
        if (enableSpike1) {
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

        // Use regular ball launch
        autoTasks.addTimedStep(() -> {}, () -> intake.launchRPMInTolerance(), 3000);
        autoTasks.addStep(intake.tasks.ballLaunchTask::restart);
        autoTasks.addStep(intake.tasks.ballLaunchTask::isDone);
        autoTasks.addDelay(1000);
        autoTasks.addStep(() -> intake.setLaunchRPM(0));
    }
}