package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.parts.intake1.DecodeSettings;
import org.firstinspires.ftc.teamcode.parts.positionsolver.settings.PositionSolverSettings;

import om.self.ezftc.utils.Vector3;
import om.self.task.other.TimedTask;

@Config
@Autonomous(name="14273.2 AutoGoalRed", group="14273")
public class T1_AutoGoalRed  extends T1_AutoFarRed {

    Integer launchRPM = 2800; // TODO: Needs Tuning.

    // Positions to travel in AutoGoalRed
    Vector3 p_targetGoal                 = new Vector3(-70.5, 70.5, 180);   // RedGoal Position.
    Vector3 p_fieldStart                = new Vector3(-39.0,55,180); // TODO: Confirm/Tune this position.
    Vector3 p_obeliskView               = new Vector3(-39.0, 31, -160);  // GoalRed: ObeliskView Position
    Vector3 p_launchPosZero             = new Vector3(-18.0,29,130);    // GoalRed Launching Position.
    Vector3 p_launchPosOne              = new Vector3(-18.0,29,130);    // GoalRed Launching Position.
    Vector3 p_launchPosTwo              = new Vector3(-18.0,29,130);    // GoalRed Launching Position for pinkServo. Z:???.

    Vector3 p_pre_intakeArtifactRow1    = new Vector3(-12, 28, -90);  // Red: Ready to collect on Row1
    Vector3 p_intakeArtifactRow1        = new Vector3(-12, 53, -90);  // Red: Intake Artifacts in Row1
    Vector3 p_pre_intakeArtifactRow2    = new Vector3(12, 28, -90);   // Red: Ready to collect on Row2
    Vector3 p_intakeArtifactRow2        = new Vector3(12, 60, -90);   // Red: Intake Artifacts in Row2

    Vector3 p_pre_intakeArtifactRow3    = new Vector3(35.5, 28, -90);   // Red: Ready to collect in Row3
    Vector3 p_intakeArtifactRow3        = new Vector3(35.5, 60, -90);   // Red: Intake Artifacts in Row3

    Vector3 p_leverOpen                 = new Vector3(0, 55, 180);    // Red: Open Lever Position
    Vector3 p_parkAfterAuto             = new Vector3(-12,28,180);

    // testNewAuto
    //     Look at Obelisk, Determine Game Classification Pattern and Store it.
    //     Launch Pre-Loaded Artifacts.
    //     Intake and Launch Artifacts in Row-1.
    //     Intake and Launch Artifacts in Row-2.
    //     Intake and Launch Artifacts in Row-3.
    //     Park!
    @Override
    protected void testNewAuto(TimedTask autoTasks) {

        // Reset and Get Ready.
        autoTasks.addStep(() -> intake.stopAllIntakeTasks());
        autoTasks.addDelay(250);
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceSettings));
        autoTasks.addStep(() -> intake.tasks.allServoStore.restart());
        autoTasks.addTimedStep(() -> {}, () -> intake.tasks.allServoStore.isDone(), 250);

        //Move to ObeliskView position.
        positionSolver.addMoveToTaskEx(DecodeSettings.getObeliskViewPos(), autoTasks, 1000);
        // Look at Obelisk, determine classificationId and Store it.
        DecodeSettings.setClassificationId(limelight.getClassificationId());

        // Prep "Launch Motor".
        autoTasks.addStep(() -> intake.setLaunchRPM((int) DecodeSettings.getLaunchRPM()));
        autoTasks.addStep(() -> intake.tasks.startGoalLaunch.restart());   // TODO: Update startAutoGoalLaunch to use intake.launchRPM.
        autoTasks.addTimedStep(() -> {}, () -> intake.launchRPMInTolerance(), 3000);

//        // Move to Launch Position.
//        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceSettings));
//        positionSolver.addMoveToTaskEx(p_LaunchPos, autoTasks);
//
        // Launch Pre-loaded Artifacts.
        //      Determine LaunchOrder and Launch.
//        autoTasks.addStep(() -> intake.computeLaunchOrderAndLaunch(DecodeSettings.getClassificationId()));
//        autoTasks.addDelay(2500);
        //      Move to LaunchPositions and launchServos in defaultOrder. (pink, blue, green).
        positionSolver.addMoveToTaskEx(DecodeSettings.getLaunchPositionTwo(), autoTasks);
        autoTasks.addStep(() -> intake.tasks.pinkServoLaunch.restart());
        autoTasks.addDelay(300);
        positionSolver.addMoveToTaskEx(DecodeSettings.getLaunchPositionOne(), autoTasks);
        autoTasks.addStep(() -> intake.tasks.blueServoLaunch.restart());
        autoTasks.addDelay(300);
        positionSolver.addMoveToTaskEx(DecodeSettings.getLaunchPositionZero(), autoTasks);
        autoTasks.addStep(() -> intake.tasks.greenServoLaunch.restart());
        autoTasks.addDelay(300);

        // Intake from Row1 and Launch.
        artifactIntakeAndLaunch(autoTasks, DecodeSettings.getPreIntakeArtifactRow1(), DecodeSettings.getIntakeArtifactRow1());

        // Intake from Row2 and Launch.
        artifactIntakeAndLaunch(autoTasks, DecodeSettings.getPreIntakeArtifactRow2(), DecodeSettings.getIntakeArtifactRow2());

        // Intake from Row3 and Launch.
//        artifactIntakeAndLaunch(autoTasks, DecodeSettings.getPreIntakeArtifactRow3(), DecodeSettings.getIntakeArtifactRow3());

        // Move to ParkAfterAuto Position.
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceSettings));
        positionSolver.addMoveToTaskEx(DecodeSettings.getParkAfterAutoPos(), autoTasks);
    }
    @Override
    public void extraSettings() {
        DecodeSettings.isDemoMode = false;
        DecodeSettings.setAuto();
        DecodeSettings.setAllianceBlue();

        DecodeSettings.setCurrentOpMode("T1_AutoGoalRed");
        DecodeSettings.setTargetGoalPos(p_targetGoal);
        DecodeSettings.setRobotPosition(p_fieldStart);
        DecodeSettings.setObeliskViewPos(p_obeliskView);
        DecodeSettings.setLaunchPositionZero(p_launchPosZero);
        DecodeSettings.setLaunchPositionOne(p_launchPosOne);
        DecodeSettings.setLaunchPositionTwo(p_launchPosTwo);
        DecodeSettings.setPreIntakeArtifactRow1(p_pre_intakeArtifactRow1);
        DecodeSettings.setIntakeArtifactRow1(p_intakeArtifactRow1);
        DecodeSettings.setPreIntakeArtifactRow2(p_pre_intakeArtifactRow2);
        DecodeSettings.setIntakeArtifactRow2(p_intakeArtifactRow2);
        DecodeSettings.setPreIntakeArtifactRow3(p_pre_intakeArtifactRow3);
        DecodeSettings.setIntakeArtifactRow3(p_intakeArtifactRow3);
        DecodeSettings.setLeverOpenPos(p_leverOpen);
        DecodeSettings.setParkAfterAutoPos(p_parkAfterAuto);
        DecodeSettings.setLaunchRPM(launchRPM);
        DecodeSettings.lkTestMode1 = false;
    }
}

