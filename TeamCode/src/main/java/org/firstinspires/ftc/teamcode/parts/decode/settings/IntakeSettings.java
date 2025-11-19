package org.firstinspires.ftc.teamcode.parts.decode.settings;

public class  IntakeSettings {

    public double motorIntakeIn =                                   0.6;
    public double motorIntakeOut =                                 -0.6;
    public double motorIntakeStop =                                   0;
    public double servoIntakeIn =                                     0;
    public double servoIntakeOut =                                    1;
    public double servoIntakeStop =                                 0.5;

    public IntakeSettings() {
    }

    public static IntakeSettings makeDefault(){
        return new IntakeSettings();
    }
}

