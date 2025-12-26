package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.parts.intake3.settings.IntakeSettings3;
import org.firstinspires.ftc.teamcode.parts.intake3.settings.IntakeSettings3.*;
import org.firstinspires.ftc.teamcode.parts.positionsolver.settings.PositionSolverSettings;

import om.self.ezftc.utils.Vector3;
import om.self.task.other.TimedTask;

@Autonomous(name="32859 Blue Launch", group="32859")
public class T3_AutoBlueLaunch extends T3_AutoBase {
    Vector3 blueLaunchStart = new Vector3(-51, -51, -126); //Start Correct
    @Override
    public void initAuto(){
        transformFunc = (v) -> v;
        fieldStartPos = blueLaunchStart;
    }

    @Override
    public void BaseAuto(TimedTask autoTasks) {
        Vector3 aprilTag = new Vector3(-30, -30, 116); //Blue: April Tag viewing position Correct
        Vector3 shootBlue1 = new Vector3(-16, -16, -139); //Blue: Shooting position Correct
        Vector3 pickupBallBlue1 = new Vector3(-13, -33, -92); //Blue: Ready to collect on first line Correct
        Vector3 pickupBallBlue1move = new Vector3(-13, -55, -92); //Blue: Ready to move on balls in first line Correct
        Vector3 pickupBallBlue2 = new Vector3(13, -33, -92); //Blue: Ready to collect on balls in second line Correct
        Vector3 pickupBallBlue2move = new Vector3(13, -60, -92); //Blue: Ready to move on balls in second line Correct
        Vector3 pickupBallBlue3 = new Vector3(34, -34, -88); //Blue: Ready to collect on balls in third line Correct
        Vector3 pickupBallBlue3move = new Vector3(34, -60, -88); // Blue: Ready to move on balls in third line Correct
        Vector3 blueLeverOpen = new Vector3(0, -55, -90); //Blue: Open Lever Position Correct

        /* *** setup Autonomous ****/
        autoTasks.addStep(() -> intake.stopAllIntakeTasks());
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultSettings));
        autoTasks.addStep(() -> odo.setPosition(blueLaunchStart));
        MoveAndLaunch(autoTasks, intake.getSettings().launchPosiMap.get("blueshoot1"));
        autoTasks.addStep(() -> intake.stopAllIntakeTasks());
        autoTasks.addStep(() -> intake.setIntakeRPM(IntakeSettings3.intakeRPM));
        positionSolver.addMoveToTaskEx(aprilTag, autoTasks);
        autoTasks.addDelay(3000);
        positionSolver.addMoveToTaskEx(shootBlue1, autoTasks);
        autoTasks.addDelay(3000);
        positionSolver.addMoveToTaskEx(pickupBallBlue1, autoTasks);
        autoTasks.addDelay(3000);
        positionSolver.addMoveToTaskEx(pickupBallBlue1move, autoTasks);
        autoTasks.addDelay(3000);
        positionSolver.addMoveToTaskEx(pickupBallBlue2, autoTasks);
        autoTasks.addDelay(3000);
        positionSolver.addMoveToTaskEx(pickupBallBlue2move, autoTasks);
        autoTasks.addDelay(3000);
        positionSolver.addMoveToTaskEx(pickupBallBlue3, autoTasks);
        autoTasks.addDelay(3000);
        positionSolver.addMoveToTaskEx(pickupBallBlue3move, autoTasks);
        autoTasks.addDelay(3000);
        positionSolver.addMoveToTaskEx(blueLeverOpen, autoTasks);
        autoTasks.addDelay(3000);
        autoTasks.addStep(() -> intake.setIntakeRPM(0));
        MoveAndLaunch(autoTasks, intake.getSettings().launchPosiMap.get("blueshoot1"));
        positionSolver.addMoveToTaskEx(blueLaunchStart, autoTasks);
    }

    /************************************************************/
    private void MoveAndLaunch(TimedTask autoTasks, LaunchData launchData) {
        int RPM = launchData.getRPM();
        Vector3 pos = launchData.getPosition();
        positionSolver.addMoveToTaskExNoWait(pos, autoTasks);
        autoTasks.addStep(() -> intake.setLaunchRPM(RPM));
        autoTasks.addTimedStep(() -> {}, () -> intake.launchRPMInTolerance(), 3000);
        autoTasks.addStep(intake.tasks.ballLaunchTask::restart);
        autoTasks.addStep(intake.tasks.ballLaunchTask::isDone);
        autoTasks.addDelay(1000);
        autoTasks.addStep(() -> intake.setLaunchRPM(0));
//        positionSolver.addMoveToTaskEx(blueLaunchStart, autoTasks);
        //we are gonna set a appropraite rpm for each shooting positions using a posiMap & ect date: 12/18
    }
}

