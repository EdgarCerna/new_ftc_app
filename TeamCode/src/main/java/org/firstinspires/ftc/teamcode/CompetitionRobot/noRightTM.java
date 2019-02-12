package org.firstinspires.ftc.teamcode.CompetitionRobot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "No Right Drop TM", group = "Autonomous")
//@Disabled
public class noRightTM extends Auto_Routines {
    @Override
    public void runOpMode() {
        // Initialize Everything...
        Auto_Init();

        // Raise Lift Motor To Release Latch
        raiseLiftMotor();

        // Drive Robot Forward 500 Ticks To Free From Latch
        moveDriveEncoder(500, 500, .2);
        while(driveMotorsBusy() && !isStopRequested()){
            telemetry.addData("Status", "Driving Forward");
            telemetry.update();
        }
        setDriveMotors(0);

        // Testing If Boomerang Routine Works
        boomerangAutoTM();

        // LOWER LIFT MOTOR TO RETURN TO NORMAL SIZE
        lowerLiftMotor();
    }
}
