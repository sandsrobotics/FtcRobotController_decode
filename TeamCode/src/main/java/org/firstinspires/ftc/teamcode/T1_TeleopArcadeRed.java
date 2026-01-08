package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.parts.intake1.DecodeSettings;

import om.self.ezftc.utils.Vector3;

@TeleOp(name="14273 TeleArcadeRed", group="B14273")
public class T1_TeleopArcadeRed extends T1_TeleopArcadeBlue {
    protected Vector3 fieldStartPos = new Vector3(64,16,180);

    @Override
    protected void extraSettings() {
        DecodeSettings.isDemoMode = false;
        DecodeSettings.setTeleOp();
        DecodeSettings.setAllianceRed();
        DecodeSettings.storeRobotPosition(fieldStartPos); // TODO: Do this only if the currentPosition is (0,0,0)?
    }
}
