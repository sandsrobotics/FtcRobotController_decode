package org.firstinspires.ftc.teamcode.depricated.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.parts.drive.DriveTeleop;
import org.firstinspires.ftc.teamcode.parts.drive.settings.DriveTeleopSettings;

@Disabled
@TeleOp(name="14273.1 Forza RED", group="14273")
public class FlipTeleopForza extends FlipTeleopDive {
    @Override
    public void initTeleop(){
        new DriveTeleop(drive, DriveTeleopSettings.makeForza1(robot));
    }
}