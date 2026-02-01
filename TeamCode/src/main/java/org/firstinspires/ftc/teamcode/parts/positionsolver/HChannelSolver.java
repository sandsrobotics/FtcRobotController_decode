
package org.firstinspires.ftc.teamcode.parts.positionsolver;

import org.firstinspires.ftc.teamcode.parts.drive.DriveControl;

import om.self.ezftc.core.part.ControllablePart;

public abstract class HChannelSolver extends Solver<HeadingSolver, DriveControl>{
    public HChannelSolver(HeadingSolver headingSolver, String name) {
        super(headingSolver, name);
    }

    @Override
    public ControllablePart<?, ?, ?, DriveControl> getControlled() {
        return parent.parent;
    }
}
