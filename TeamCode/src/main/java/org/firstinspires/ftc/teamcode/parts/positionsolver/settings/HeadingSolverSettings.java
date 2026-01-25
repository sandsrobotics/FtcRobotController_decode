
package org.firstinspires.ftc.teamcode.parts.positionsolver.settings;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class HeadingSolverSettings {
    public final SolverSettings rChannelSettings;

    public static PIDCoefficients OmRotate = new PIDCoefficients(0.0125,0.00,0.00);
    public static PIDCoefficients LKRotate = new PIDCoefficients(0.026,0.01,0.00025);

    public HeadingSolverSettings(SolverSettings rChannelSettings) {
        this.rChannelSettings = rChannelSettings;
    }

    public static HeadingSolverSettings defaultSettings = new HeadingSolverSettings(
            new SolverSettings(1.5, 10, true, 10000, OmRotate, 1)
    );

    public static HeadingSolverSettings looseSettings = new HeadingSolverSettings(
            new SolverSettings(5, 10, true, 10000, OmRotate, 1)
    );

    public static HeadingSolverSettings extraLoseSettings = new HeadingSolverSettings(
            new SolverSettings(0.5, 10, true, 10000, OmRotate, 1)
    );

}
