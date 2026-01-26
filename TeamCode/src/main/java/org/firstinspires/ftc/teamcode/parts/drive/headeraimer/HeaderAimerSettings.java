package org.firstinspires.ftc.teamcode.parts.drive.headeraimer;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class HeaderAimerSettings {
    public final double minRegisterVal;

    public final PIDCoefficients pidCoefficients;
//    public final double maxI;

//    public final int headingSettleDelay;

    public HeaderAimerSettings(double minRegisterVal, PIDCoefficients pidCoefficients) { //}, double maxI, int headingSettleDelay) {
        this.minRegisterVal = minRegisterVal;
        this.pidCoefficients = pidCoefficients;
//        this.maxI = maxI;
//        this.headingSettleDelay = headingSettleDelay;
    }

    public static HeaderAimerSettings makeDefault(){
        return new HeaderAimerSettings(
                0.01,
                new PIDCoefficients(
                        0.03,0,0   // was -0.02
                )//,
//                0.3,
//                700
        );
    }
}
