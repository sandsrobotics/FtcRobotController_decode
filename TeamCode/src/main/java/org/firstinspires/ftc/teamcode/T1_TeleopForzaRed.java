package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.parts.drive.DriveTeleop;
import org.firstinspires.ftc.teamcode.parts.drive.settings.DriveTeleopSettings;
import org.firstinspires.ftc.teamcode.parts.intake1.DecodeSettings;

@TeleOp(name="14273 TeleForzaRed", group="B14273")
public class T1_TeleopForzaRed extends T1_TeleopArcadeRed {
    @Override
    public void initTeleop(){
        new DriveTeleop(drive, DriveTeleopSettings.makeForza1(robot));
    }

    @Override
    protected void extraSettings() {
        super.extraSettings();
        DecodeSettings.setCurrentOpMode("T1_TeleopForzaRed");
    }

}
