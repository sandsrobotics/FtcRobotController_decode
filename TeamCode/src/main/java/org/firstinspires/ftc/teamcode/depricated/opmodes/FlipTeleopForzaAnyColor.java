package org.firstinspires.ftc.teamcode.depricated.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.parts.intake.FlipbotSettings;
@Disabled
@TeleOp(name="14273.3 Forza ANY COLOR", group="14273")
public class FlipTeleopForzaAnyColor extends FlipTeleopForza {
    @Override
    public void extraSettings() {
        FlipbotSettings.isBlueGood = true;
        FlipbotSettings.isYellowGood = true;
        FlipbotSettings.isRedGood = true;
        FlipbotSettings.isEverythingGood = true;
    }
}