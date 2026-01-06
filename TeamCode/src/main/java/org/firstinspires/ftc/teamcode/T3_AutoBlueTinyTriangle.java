package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.parts.intake3.settings.IntakeSettings3;
import org.firstinspires.ftc.teamcode.parts.intake3.settings.IntakeSettings3.*;
import org.firstinspires.ftc.teamcode.parts.positionsolver.settings.PositionSolverSettings;
import om.self.ezftc.utils.Vector3;
import om.self.task.other.TimedTask;

@Autonomous(name="Blue TinyTriangle", group="32859")
public class T3_AutoBlueTinyTriangle extends T3_AutoBase {
    Vector3 blueLaunchStart = new Vector3(64, -6, 180);; //to do make new start pos
    @Override
    public void initAuto(){
        transformFunc = (v) -> v;
        fieldStartPos = blueLaunchStart;
    }

    @Override
    public void BaseAuto(TimedTask autoTasks) {
        Vector3 start = fieldStartPos; //new Vector3(-51, -50, 140);
        Vector3 aprilTag = transformFunc.apply(new Vector3(-42, -41, 140)); //may already be sensed
        //red shoot 49,16,160
        Vector3 shoot = new Vector3(-16, -16, -139); //already good
        IntakeSettings3.LaunchData shootLaunchData = new IntakeSettings3.LaunchData(3225, transformFunc.apply(new Vector3(-11, -11, -139)));
        Vector3 blueSpikeReady1 = transformFunc.apply(new Vector3(-12,-28,-90));
        Vector3 blueSpike1 = transformFunc.apply(new Vector3(-12,-48,-90));
        Vector3 blueSpikeReady2 = transformFunc.apply(new Vector3(12,-28,-90));
        Vector3 blueSpike2 = transformFunc.apply(new Vector3(12,-60,-90));
        Vector3 blueSpikeReady3 = transformFunc.apply(new Vector3(34.5,-28,-90));
        Vector3 blueSpike3 = transformFunc.apply(new Vector3(34,-60,-90));

        /* *** setup Autonomous ****/
        autoTasks.addStep(() -> intake.stopAllIntakeTasks());
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultSettings));
        autoTasks.addStep(() -> odo.setPosition(start));
        positionSolver.addMoveToTaskEx(aprilTag, autoTasks);
//        autoTasks.addDelay(1500);
        // validate april tag is seen

        MoveAndLaunch(autoTasks, shootLaunchData);
//        autoTasks.addDelay(1500);
//        MoveAndLaunch(autoTasks, shootLaunchData);
//        autoTasks.addDelay(1500);
        autoTasks.addStep(() -> intake.setIntakeRPM(IntakeSettings3.intakeRPM));
        positionSolver.addMoveToTaskEx(blueSpikeReady3, autoTasks);
//        autoTasks.addDelay(1500);

        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.ultraSlowSettings));
        positionSolver.addMoveToTaskEx(blueSpike3, autoTasks);
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultSettings));
        autoTasks.addDelay(1000);
        autoTasks.addStep(() -> intake.setIntakeRPM(0));

        MoveAndLaunch(autoTasks, shootLaunchData);
//        autoTasks.addDelay(1500);
        positionSolver.addMoveToTaskEx(blueSpikeReady2, autoTasks);
//        autoTasks.addDelay(1500);
        autoTasks.addStep(() -> intake.setIntakeRPM(IntakeSettings3.intakeRPM));

        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.ultraSlowSettings));
        positionSolver.addMoveToTaskEx(blueSpike2, autoTasks);
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultSettings));
        positionSolver.addMoveToTaskExNoWait(blueSpikeReady2, autoTasks);
        autoTasks.addDelay(1000);
        autoTasks.addStep(() -> intake.setIntakeRPM(0));

        autoTasks.addStep(() -> intake.setIntakeRPM(IntakeSettings3.intakeRPM));
        positionSolver.addMoveToTaskEx(blueSpikeReady1, autoTasks);

//        autoTasks.addDelay(1500);
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.ultraSlowSettings));
        positionSolver.addMoveToTaskEx(blueSpike1, autoTasks);
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultSettings));
        autoTasks.addDelay(1000);  // do this with a no wait style move task
        autoTasks.addStep(() -> intake.setIntakeRPM(0));

        MoveAndLaunch(autoTasks, shootLaunchData);
//        autoTasks.addDelay(1500);
        autoTasks.addStep(() -> intake.setIntakeRPM(0));
    }

    /************************************************************/
    private void MoveAndLaunch(TimedTask autoTasks, IntakeSettings3.LaunchData launchData) {
        int RPM = launchData.getRPM();
        Vector3 pos = launchData.getPosition();
        positionSolver.addMoveToTaskExNoWait(pos, autoTasks);
        autoTasks.addStep(() -> intake.setLaunchRPM(RPM));
        autoTasks.addTimedStep(() -> {}, () -> intake.launchRPMInTolerance(), 3000);
        autoTasks.addStep(intake.tasks.ballLaunchTask::restart);
        autoTasks.addStep(intake.tasks.ballLaunchTask::isDone);
        autoTasks.addDelay(1000);
        autoTasks.addStep(() -> intake.setLaunchRPM(0));
        //we are gonna set a appropraite rpm for each shooting positions using a posiMap & ect date: 12/18
    }
}


