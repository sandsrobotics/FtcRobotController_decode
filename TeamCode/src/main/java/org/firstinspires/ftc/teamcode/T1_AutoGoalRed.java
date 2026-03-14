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

    Integer launchRPM = 2450; // 2350; 2500; TODO: Needs Tuning.

    // Positions to travel in AutoGoalRed
    Vector3 p_targetGoal                = new Vector3(-70.5, 60.5, 180);   // Y: 70.5; RedGoal Position.
    Vector3 p_fieldStart                = new Vector3(-41.5,55,180); // (-40.0, 55, 180); X: -39.0; TODO: Confirm/Tune this position.
    Vector3 p_obeliskView               = new Vector3(-39.0, 31, -160);  // GoalRed: ObeliskView Position
    Vector3 p_launchPosZero             = new Vector3(-16.0,16,132);    // GoalRed Launching Position.
    Vector3 p_launchPosOne              = new Vector3(-16.0,16,132);    // 135; GoalRed Launching Position.
    Vector3 p_launchPosTwo              = new Vector3(-16.0,16,135);    // Z:127; (-28,16,124) Z: 131; 135; Was: -18, 29, 130? GoalRed Launching Position for pinkServo.

    Vector3 p_pre_intakeArtifactRow1    = new Vector3(-12, 22, -90);  // Red: Ready to collect on Row1
    Vector3 p_intakeArtifactRow1        = new Vector3(-12, 53, -90);  // Red: Intake Artifacts in Row1
    Vector3 p_pre_intakeArtifactRow2    = new Vector3(14, 22, -90);   // Red: Ready to collect on Row2
    Vector3 p_intakeArtifactRow2        = new Vector3(14, 60, -90);   // Red: Intake Artifacts in Row2

    Vector3 p_pre_intakeArtifactRow3    = new Vector3(35.5, 22, -90);   // Red: Ready to collect in Row3
    Vector3 p_intakeArtifactRow3        = new Vector3(35.5, 60, -90);   // Red: Intake Artifacts in Row3

    Vector3 p_pre_leverOpen             = new Vector3(-4, 45, 180);    // X:0; Red: Open Lever Position
    Vector3 p_leverOpen                 = new Vector3(-4, 55, 180);    // X:0; Red: Open Lever Position
    Vector3 p_parkAfterAuto             = new Vector3(-9,28,180);

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
        autoTasks.addStep(() -> intake.tasks.allServoStore.restart());

        //Move to ObeliskView position.
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceSettings));
        positionSolver.addMoveToTaskEx(DecodeSettings.getObeliskViewPos(), autoTasks, 1000);
        // Look at Obelisk, determine classificationId and Store it.
        autoTasks.addStep(intake.tasks.viewObelisk::restart);

        // Prep "Launch Motor".
        autoTasks.addStep(() -> intake.setLaunchRPM((int) DecodeSettings.getLaunchRPM()));
        autoTasks.addStep(() -> intake.tasks.startGoalLaunch.restart());   // TODO: Create/Update startGoalLaunch to use intake.launchRPM.

        // Move to Launch Position.
        positionSolver.addMoveToTaskEx(DecodeSettings.getLaunchPositionTwo(), autoTasks);
        autoTasks.addStep(() -> intake.tasks.startGoalLaunch.isDone());

        // Determine LaunchOrder and Launch
        autoTasks.addStep(() -> intake.tasks.nearComputeAndLaunchInOrder.restart());
        autoTasks.addStep(() -> intake.tasks.nearComputeAndLaunchInOrder.isDone());

//        autoTasks.addStep(() -> intake.tasks.computeAndLaunchInOrder.restart());
//        autoTasks.addStep(() -> intake.tasks.computeAndLaunchInOrder.isDone());

        // Intake from Row1.
        if (runSpikeCount >=1 ) {
            artifactIntake(autoTasks, DecodeSettings.getPreIntakeArtifactRow1(), DecodeSettings.getIntakeArtifactRow1());

            // Clear Ramp?
            if (runLeverOpen == 1) {
                clearRamp(autoTasks);
            }

            // ArtifactLaunch.
            artifactLaunch(autoTasks);
        }


        // Intake from Row2.
        if (runSpikeCount >=2 ) {
            artifactIntake(autoTasks, DecodeSettings.getPreIntakeArtifactRow2(), DecodeSettings.getIntakeArtifactRow2());
            // ArtifactLaunch.
            artifactLaunch(autoTasks);
        }

        // Intake from Row3 and Launch.
        if (runSpikeCount >=3) {
            artifactIntake(autoTasks, DecodeSettings.getPreIntakeArtifactRow3(), DecodeSettings.getIntakeArtifactRow3());
            // ArtifactLaunch.
            artifactLaunch(autoTasks);
        }

        // Move to ParkAfterAuto Position.
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceSettings));
        positionSolver.addMoveToTaskEx(DecodeSettings.getParkAfterAutoPos(), autoTasks);
    }

    // Artifact Intake.
    protected void artifactIntake (TimedTask autoTasks,
                                   Vector3 pos_pre_intake,
                                   Vector3 pos_intake) {
        // Move to pre_intake position.
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceSlowSettings));
        positionSolver.addMoveToTaskEx(pos_pre_intake, autoTasks);

        // Start "intake".
        autoTasks.addStep(() -> intake.tasks.intakeTask.restart());
        autoTasks.addStep(() -> intake.tasks.intakeTask.isDone());

        // Set positionSolver to "ExtraSlow" to allow intake slowly.
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceExtraSlowSettings));

        // Move to intake.
        positionSolver.addMoveToTaskEx(pos_intake, autoTasks, 2000);
        autoTasks.addDelay(1500); // 1000; 1500; 2500; Test with 1000.
    }

    // Artifact Launch.
    protected void artifactLaunch (TimedTask autoTasks) {

        // Move to launch.
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceSlowSettings));
        positionSolver.addMoveToTaskEx(DecodeSettings.getLaunchPositionTwo(), autoTasks);
        autoTasks.addStep(() -> intake.tasks.artifactIntakeStopTask.restart());
        autoTasks.addStep(() -> intake.tasks.artifactIntakeStopTask.isDone());
        //  Determine LaunchOrder and Launch
//        autoTasks.addStep(() -> intake.tasks.nearComputeAndLaunchInOrder.restart());
//        autoTasks.addStep(() -> intake.tasks.nearComputeAndLaunchInOrder.isDone());
        autoTasks.addStep(() -> intake.tasks.computeAndLaunchInOrder.restart());
        autoTasks.addStep(() -> intake.tasks.computeAndLaunchInOrder.isDone());

    }

    // clearRamp.
    protected void clearRamp (TimedTask autoTasks) {

        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceSlowSettings));
        // Move to pre-clearRamp.
        positionSolver.addMoveToTaskEx(DecodeSettings.getPreLeverOpenPos(), autoTasks, 2000);
        autoTasks.addStep(()-> intake.tasks.artifactIntakeStopTask.restart());

        // Move to clearRamp.
        positionSolver.addMoveToTaskEx(DecodeSettings.getLeverOpenPos(), autoTasks, 1000);

        // Wait for Artifacts to Clear.
        autoTasks.addDelay(1500);

        // Move to pre-clearRamp.
        positionSolver.addMoveToTaskEx(DecodeSettings.getPreLeverOpenPos(), autoTasks, 1500);
    }

    @Override
    public void extraSettings() {
        DecodeSettings.isDemoMode = false;
        DecodeSettings.setAuto();
        DecodeSettings.setAllianceRed();

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
        DecodeSettings.setPreLeverOpenPos(p_pre_leverOpen);
        DecodeSettings.setLeverOpenPos(p_leverOpen);
        DecodeSettings.setParkAfterAutoPos(p_parkAfterAuto);
        DecodeSettings.setLaunchRPM(launchRPM);
        DecodeSettings.lkTestMode1 = false;
    }
}

