package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.parts.intake1.DecodeSettings;

import om.self.ezftc.utils.Vector3;

@Disabled
@TeleOp(name="14273 TeleArcadeRed", group="B14273")
public class T1_TeleopArcadeRed extends T1_TeleopArcadeBlue {
    protected Vector3 p_targetGoal  = new Vector3(-70.5, 70.5, 180);   // Y: 60.5; Y: 70.5 RedGoal Position.
    protected Vector3 fieldStartPos = new Vector3(64,16,180);

    @Override
    protected void extraSettings() {
        DecodeSettings.isDemoMode = false;
        DecodeSettings.setTeleOp();
        DecodeSettings.setAllianceRed();
        DecodeSettings.setCurrentOpMode("T1_TeleopArcadeRed");
        DecodeSettings.setTargetGoalPos(p_targetGoal);

        // Reset to fieldStartPos only when it's zero. Otherwise, Carry-over Position from Auto.
        Vector3 tempPosition = DecodeSettings.getRobotPosition();
        if (tempPosition.X == 0.0 && tempPosition.Y == 0.0 && tempPosition.Z == 0.0)  {
            DecodeSettings.setRobotPosition(fieldStartPos);
            DecodeSettings.odoFirstRun = true;
        }
        DecodeSettings.lkTestMode1 = true;
    }
}
