package org.firstinspires.ftc.teamcode.CompetitionRobot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.Locale;

@Autonomous(name = "DeployArm", group = "Autonomous")
//@Disabled
public class testDeployArm extends Auto_Routines {
    @Override
    public void runOpMode() {
        // Initialize Everything...
        Auto_Init();

        hoverArm(1000, 0.3);
    }
}
