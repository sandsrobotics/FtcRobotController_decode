package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.parts.intake3.settings.IntakeSettings3.*;
import org.firstinspires.ftc.teamcode.parts.positionsolver.settings.PositionSolverSettings;
import java.util.Map;
import om.self.ezftc.utils.Vector3;
import om.self.task.other.TimedTask;

@Autonomous(name="32859 Blue Launch", group="32859")
public class T3_AutoBlueLaunch extends T3_AutoBase {
    Vector3 blueLaunchStart = new Vector3(-51, 51, 143); //Start
    @Override
    public void initAuto(){
        transformFunc = (v) -> v;
        fieldStartPos = blueLaunchStart;
    }

    @Override
    public void BaseAuto(TimedTask autoTasks) {
        //Vector3 shootRed = new Vector3(12, 12, 45);
        Vector3 shootBlue1 = new Vector3(-12, 12, 135); //Shooting position
        Vector3 pickupBallBlue1 = new Vector3(-24, 10, 180); //Ready to collect on first line
        Vector3 pickupBallBlue2 = new Vector3(-53, 12, 180); //Move in on balls in first line
        /* *** setup Autonomous ****/
        autoTasks.addStep(() -> intake.stopAllIntakeTasks());
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultSettings));
        autoTasks.addStep(() -> odo.setPosition(blueLaunchStart));

        MoveAndLaunch(autoTasks, intake.getSettings().launchPosiMap.get("blueshoot1"));
        autoTasks.addStep(() -> intake.stopAllIntakeTasks());
        autoTasks.addStep(() -> intake.setIntakeRPM(1000));
        positionSolver.addMoveToTaskEx(pickupBallBlue1, autoTasks);
        positionSolver.addMoveToTaskEx(pickupBallBlue2, autoTasks);
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
        //autoTasks.addStep(intake.tasks.ballLaunchTask::isDone);
        autoTasks.addDelay(1000);
        autoTasks.addStep(() -> intake.setLaunchRPM(0));
//        positionSolver.addMoveToTaskEx(blueLaunchStart, autoTasks);
    }
}

