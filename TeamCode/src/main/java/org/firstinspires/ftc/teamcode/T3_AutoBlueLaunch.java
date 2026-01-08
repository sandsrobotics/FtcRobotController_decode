package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.parts.intake3.settings.IntakeSettings3;
import org.firstinspires.ftc.teamcode.parts.intake3.settings.IntakeSettings3.*;
import org.firstinspires.ftc.teamcode.parts.positionsolver.settings.PositionSolverSettings;

import om.self.ezftc.utils.Vector3;
import om.self.task.other.TimedTask;

@Autonomous(name="Blue Goal", group="32859")
public class T3_AutoBlueLaunch extends T3_AutoBase {
    Vector3 blueLaunchStart = new Vector3(-58.8,-45,143);; //-51, -50, 140
    @Override
    public void initAuto(){
        transformFunc = (v) -> v;
        fieldStartPos = blueLaunchStart;
    }

    //        autoTasks.addDelay(1500);
    //        autoTasks.addStep(() -> intake.setIntakeRPM(-500)); // for reverse intake

    @Override
    public void BaseAuto(TimedTask autoTasks) {
        Vector3 start = fieldStartPos; //new Vector3(-51, -50, 140);
        Vector3 aprilTag = transformFunc.apply(new Vector3(-42, -41, 140)); //z=122

        LaunchData shootLaunchData = new LaunchData(3225, transformFunc.apply(new Vector3(-11, -11, -139)));

        Vector3 blueSpikeReady1 = transformFunc.apply(new Vector3(-12,-28,-90));
        Vector3 blueSpike1 = transformFunc.apply(new Vector3(-12,-53,-90));
        Vector3 blueSpikeReady2 = transformFunc.apply(new Vector3(12,-28,-90));
        Vector3 blueSpike2 = transformFunc.apply(new Vector3(12,-60,-90));
        Vector3 blueSpikeReady3 = transformFunc.apply(new Vector3(35.5,-28,-90)); //x was 35, needed to go left more
        Vector3 blueSpike3 = transformFunc.apply(new Vector3(35.5,-60,-90)); //x was 35, needed to go left more

        /* *** setup Autonomous ****/
        autoTasks.addStep(() -> intake.stopAllIntakeTasks());
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultSettings));
        autoTasks.addStep(() -> odo.setPosition(start));

        positionSolver.addMoveToTaskEx(aprilTag, autoTasks);

        MoveAndLaunch(autoTasks, shootLaunchData);
        autoTasks.addStep(() -> intake.setIntakeRPM(IntakeSettings3.intakeRPM));
        positionSolver.addMoveToTaskEx(blueSpikeReady1, autoTasks);
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.ultraSlowSettings));
        positionSolver.addMoveToTaskEx(blueSpike1, autoTasks);
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultSettings));
        autoTasks.addDelay(500);  // finish intakeing

        MoveAndLaunch(autoTasks, shootLaunchData);
        autoTasks.addStep(() -> intake.setIntakeRPM(IntakeSettings3.intakeRPM));
        positionSolver.addMoveToTaskEx(blueSpikeReady2, autoTasks);
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.ultraSlowSettings));
        positionSolver.addMoveToTaskEx(blueSpike2, autoTasks);
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultSettings));
        autoTasks.addDelay(500);
        positionSolver.addMoveToTaskEx(blueSpikeReady2, autoTasks);

        MoveAndLaunch(autoTasks, shootLaunchData);
        autoTasks.addStep(() -> intake.setIntakeRPM(IntakeSettings3.intakeRPM));
        positionSolver.addMoveToTaskEx(blueSpikeReady3, autoTasks);
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.ultraSlowSettings));
        positionSolver.addMoveToTaskEx(blueSpike3, autoTasks);
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultSettings));
        autoTasks.addDelay(500);

        MoveAndLaunch(autoTasks, shootLaunchData);
        autoTasks.addStep(() -> intake.setIntakeRPM(0));
        autoTasks.addStep(() -> intake.setLaunchRPM(0));
    }

    /************************************************************/
    private void MoveAndLaunch(TimedTask autoTasks, LaunchData launchData) {
        int RPM = launchData.getRPM();
        Vector3 pos = launchData.getPosition();
        positionSolver.addMoveToTaskExNoWait(pos, autoTasks);
        autoTasks.addStep(() -> intake.setLaunchRPM(RPM));
        autoTasks.addTimedStep(()-> intake.setIntakeRPM(-500),()->positionSolver.isDone(),3000);
        autoTasks.addStep(() -> intake.setIntakeRPM(0));
//        autoTasks.addTimedStep(() -> {}, () -> intake.launchRPMInTolerance(), 4000);
        autoTasks.addStep(intake.tasks.ballLaunchTask::restart);
        autoTasks.addStep(intake.tasks.ballLaunchTask::isDone);

        //This dan go away once we have hardware to let launcher on all time
//        autoTasks.addStep(() -> intake.setLaunchRPM(0));
    }
}

