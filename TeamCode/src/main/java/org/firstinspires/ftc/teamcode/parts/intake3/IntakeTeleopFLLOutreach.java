    package org.firstinspires.ftc.teamcode.parts.intake3;

    import com.qualcomm.robotcore.hardware.DcMotorEx;

    import org.apache.commons.lang3.ObjectUtils;
    import org.firstinspires.ftc.teamcode.lib.ButtonMgr;
    import org.firstinspires.ftc.teamcode.lib.ButtonMgr.Buttons;
    import org.firstinspires.ftc.teamcode.lib.ButtonMgr.State;
    import org.firstinspires.ftc.teamcode.parts.intake3.settings.IntakeSettings3;
    import org.firstinspires.ftc.teamcode.parts.positionsolver.settings.PositionSolverSettings;

    import om.self.ezftc.core.part.LoopedPartImpl;
    import om.self.ezftc.utils.Vector3;


    //@Config
    public class IntakeTeleopFLLOutreach extends LoopedPartImpl<Intake3, IntakeSettings3, ObjectUtils.Null> {
        private IntakeSettings3 settings;
        ButtonMgr buttonMgr;
        private boolean c1Align;
        private boolean c2Align;
        private boolean allowC2Launch = false;



        public IntakeTeleopFLLOutreach(Intake3 parent) {
            super(parent, "Intake teleop FLLOutreach");
            setSettings(IntakeSettings3.makeDefault());
            buttonMgr = parent.parent.buttonMgr;
        }

        public IntakeTeleopFLLOutreach(Intake3 parent, IntakeSettings3 settings) {
            super(parent, "Intake teleop FLLOutreach");
            setSettings(settings);
            buttonMgr = parent.parent.buttonMgr;
        }

        public IntakeSettings3 getSettings() {
            return settings;
        }

        public void setSettings(IntakeSettings3 settings) {
            this.settings = settings;
        }

        @Override
        public void onBeanLoad() {}

        @Override
        public void onInit() {}

        @Override
        public void onStart() {
            parent.setBaseController(() -> new IntakeControl3(false), true);
            parent.getHardware().launchMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, IntakeSettings3.spinnerPID);
        }

        @Override
        public void onRun() {
            driverControls();
        }

        @Override
        public void onStop() {
            parent.setBaseControllerToDefault(parent.isControlActive());
        }

        /* **********************************************************
         *
         * IMPORTANT NOTE FROM MR. KN:
         *
         * State.isPressed should not be used when controlling tasks (or triggering anything).
         * It returns the current state of the button, not a rising or falling edge event.
         * That means it triggers the task repeatedly for the entire duration it is pressed.
         * Not good!
         *
         * Better choices would be:
         *    wasPressed;       // Rise
         *    wasReleased;      // Fall
         *    wasHeld;          // Rise + Long Hold + Fall
         *    wasTapped;        // Rise + Short Hold + Fall
         *    wasSingleTapped;  // Rise + Short Hold + Fall + Gap  [note the gap/delay at end!]
         *    wasDoubleTapped;  // Rise + Short Hold + Fall + Short Gap x2
         *
         * ***********************************************************/

        public void driverControls() {

            // ============================================
            // CONTROLLER 1 (DRIVER) CONTROLS
            // ============================================

            // BACK BUTTON - Emergency stop
            if (buttonMgr.getState(1, Buttons.back, State.wasPressed)) {
                parent.eStop();
            }

            // RIGHT BUMPER - Allow Controller 2 to launch
            if (buttonMgr.getState(1, Buttons.right_bumper, State.isHeld)) {
                allowC2Launch = true;
            } else {
                allowC2Launch = false;
            }
            // D-PAD DOWN - Start intake (collect artifacts)
            if (buttonMgr.getState(1, Buttons.dpad_down, State.wasTapped)) {
                parent.setIntakeRPM(IntakeSettings3.intakeRPM);
            }

            // D-PAD UP - Reverse intake (eject artifacts)
            if (buttonMgr.getState(1, Buttons.dpad_up, State.wasTapped)) {
                parent.setIntakeRPM(-IntakeSettings3.intakeRPM);
            }

            // D-PAD DOWN/UP DOUBLE TAP - Stop intake
            if (buttonMgr.getState(1, Buttons.dpad_down, State.wasDoubleTapped) ||
                    buttonMgr.getState(1, Buttons.dpad_up, State.wasDoubleTapped)) {
                parent.setIntakeRPM(0);
            }

            // RIGHT TRIGGER - Color-ordered launch (auto-starts launcher)
            if (buttonMgr.getState(1, Buttons.right_trigger, State.wasPressed)) {
                if (parent.getTargetLaunchRPM() < 500) {
                    parent.setLaunchRPM(IntakeSettings3.launchRPM);
                    IntakeSettings3.launchArmed = true;
                }
                parent.stopAllIntakeTasks();
                parent.tasks.orderedColorLaunchTask.restart();
            }

            // LEFT BUMPER - Auto-align to April tag
            if (buttonMgr.getState(1, Buttons.left_bumper, State.isHeld) ||
                    buttonMgr.getState(2, Buttons.left_bumper, State.isHeld) ) {
                c1Align = true;
                // IntakeSettings3.alignTarget = true;
            } else {
                c1Align = false;
                // IntakeSettings3.alignTarget = false;
            }

            // Y BUTTON - Start/Stop launcher
            if (buttonMgr.getState(1, Buttons.y, State.wasTapped)) {
                parent.setLaunchRPM(IntakeSettings3.launchRPM);
                IntakeSettings3.launchArmed = true;  // use true for interpolated
            }

            if (buttonMgr.getState(1, Buttons.y, State.wasDoubleTapped)) {
                IntakeSettings3.launchArmed = false;
                parent.setLaunchRPM(0);
            }

            if (!buttonMgr.getState(1, Buttons.left_trigger, State.isHeld)) {


                // ============================================
                // CONTROLLER 2 (OPERATOR) CONTROLS
                // ============================================

                // BACK BUTTON - Emergency stop
                if (buttonMgr.getState(2, Buttons.back, State.wasPressed)) {
                    parent.eStop();
                }

                // D-PAD DOWN - Start intake (collect artifacts)
                if (buttonMgr.getState(2, Buttons.dpad_down, State.wasTapped)) {
                    parent.setIntakeRPM(IntakeSettings3.intakeRPM);
                }
                // D-PAD UP - Reverse intake (eject artifacts)
                if (buttonMgr.getState(2, Buttons.dpad_up, State.wasTapped)) {
                    parent.setIntakeRPM(-IntakeSettings3.intakeRPM);
                }

                // D-PAD DOWN/UP DOUBLE TAP - Stop intake
                if (buttonMgr.getState(2, Buttons.dpad_down, State.wasDoubleTapped) ||
                        buttonMgr.getState(2, Buttons.dpad_up, State.wasDoubleTapped)) {
                    parent.setIntakeRPM(0);
                }

                // RIGHT TRIGGER - Color-ordered launch (auto-starts launcher)
                if (allowC2Launch) { // set by controller 1 holding right trigger
                    if (buttonMgr.getState(2, Buttons.right_trigger, State.wasPressed)) {
                        if (parent.getTargetLaunchRPM() < 500) {
                            parent.setLaunchRPM(IntakeSettings3.launchRPM);
                            IntakeSettings3.launchArmed = true;
                        }
                        parent.stopAllIntakeTasks();
                        parent.tasks.orderedColorLaunchTask.restart();
                    }
                }


    //        // LEFT BUMPER - Auto-align to April tag see above
    //        if (buttonMgr.getState(2, Buttons.left_bumper, State.isHeld)) {
    //            c2Align = true;
    //            // IntakeSettings3.alignTarget = true;
    //        } else {
    //            c2Align = false;
    //            // IntakeSettings3.alignTarget = false;
    //        }

                // Y BUTTON - Start/Stop launcher
                if (buttonMgr.getState(2, Buttons.y, State.wasTapped)) {
                    parent.setLaunchRPM(IntakeSettings3.launchRPM);
                    IntakeSettings3.launchArmed = true;  // use true for interpolated
                }

                if (buttonMgr.getState(2, Buttons.y, State.wasDoubleTapped)) {
                    IntakeSettings3.launchArmed = false;
                    parent.setLaunchRPM(0);
                }
            }
            IntakeSettings3.alignTarget = c1Align || c2Align;

        }
    }