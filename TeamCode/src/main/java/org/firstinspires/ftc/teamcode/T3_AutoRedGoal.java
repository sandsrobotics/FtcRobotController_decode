package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import om.self.ezftc.utils.Vector3;

@Autonomous(name="Red Goal", group="32859")
public class T3_AutoRedGoal extends T3_AutoBlueGoal {
    @Override
    public void initAuto() {
        isRedSide = true;
        transformFunc = (v) -> new Vector3(v.X, -v.Y, -v.Z);
        fieldStartPos = transformFunc.apply(blueLaunchStart);
    }
}
