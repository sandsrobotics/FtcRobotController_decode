package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.parts.intake1.DecodeSettings;

import om.self.ezftc.utils.Vector3;

@Config
@Autonomous(name="14273.1 AutoGoalBlue", group="14273")
public class T1_AutoGoalBlue  extends T1_AutoGoalRed {

    Integer launchRPM = 2500; // TODO: Needs Tuning.

    // Positions to travel in AutoFarBlue
    Vector3 p_targetGoal                 = new Vector3(-70.5, -70.5, 180);   // BlueGoal Position.
    Vector3 p_fieldStart                = new Vector3(-39.0,-55,-180); // TODO: Confirm/Tune this position.
    Vector3 p_obeliskView               = new Vector3(-39.0, -31, 160);  // GoalBlue: ObeliskView Position
    Vector3 p_launchPosZero             = new Vector3(-18.0,-29,-130);    // GoalBlue Launching Position.
    Vector3 p_launchPosOne              = new Vector3(-18.0,-29,-130);    // GoalBlue Launching Position.
    Vector3 p_launchPosTwo              = new Vector3(-18.0,-29,-130);    // GoalBlue Launching Position for pinkServo. Z:???.

    Vector3 p_pre_intakeArtifactRow1    = new Vector3(-12, -28, 90);  // Blue: Ready to collect on Row1
    Vector3 p_intakeArtifactRow1        = new Vector3(-12, -53, 90);  // Blue: Intake Artifacts in Row1
    Vector3 p_pre_intakeArtifactRow2    = new Vector3(12, -28, 90);   // Blue: Ready to collect on Row2
    Vector3 p_intakeArtifactRow2        = new Vector3(12, -60, 90);   // Blue: Intake Artifacts in Row2

    Vector3 p_pre_intakeArtifactRow3    = new Vector3(35.5, -28, 90);   // Blue: Ready to collect in Row3
    Vector3 p_intakeArtifactRow3        = new Vector3(35.5, -60, 90);   // Blue: Intake Artifacts in Row3

    Vector3 p_leverOpen                 = new Vector3(0, -55, -180);    // Blue: Open Lever Position
    Vector3 p_parkAfterAuto             = new Vector3(-12,-28,-180);

    @Override
    public void extraSettings() {
        DecodeSettings.isDemoMode = false;
        DecodeSettings.setAuto();
        DecodeSettings.setAllianceBlue();

        DecodeSettings.setCurrentOpMode("T1_AutoGoalBlue");
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

