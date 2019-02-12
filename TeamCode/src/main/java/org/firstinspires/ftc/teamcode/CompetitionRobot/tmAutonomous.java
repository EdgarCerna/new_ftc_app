package org.firstinspires.ftc.teamcode.CompetitionRobot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Team Marker Autonomous", group = "Autonomous")
//@Disabled
public class tmAutonomous extends Auto_Routines {
    @Override
    public void runOpMode() {
        // INITIALIZE EVERYTHING
        Auto_Init();

        // RAISE LIFT MOTOR TO RELEASE LATCH
        raiseLiftMotor();

        // DRIVE ROBOT FORWARD 500 TICKS TO ALLOW LIFT TO LOWER
        moveDriveEncoder(500, 500, .2);
        while(driveMotorsBusy() && !isStopRequested()){
            telemetry.addData("Status", "Driving Forward");
            telemetry.update();
        }
        setDriveMotors(0);

        // TESTING IF NEW ROUTINE WORKS USING GOLD SAMPLE
        tmRoutine();

        // LOWER LIFT MOTOR TO RETURN TO NORMAL SIZE
        lowerLiftMotor();
    }
}
