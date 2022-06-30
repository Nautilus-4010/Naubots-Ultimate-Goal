/*
Copyright 2019 FIRST Tech Challenge Team 4010

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Foundation Red Alliance", group="Linear Opmode")

public class NaubotsAutonomo extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private LaBarca naubot = new LaBarca(this);

    @Override
    public void runOpMode() {

        naubot.getHardware(hardwareMap);
        naubot.resetEncoders();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        naubot.moverDistanciaRecta(-30);
        naubot.girarEnEje(-30);
        naubot.moverDistanciaRecta(-80);
        naubot.girarEnEje(20);
        naubot.moverDistanciaRecta(-40);
        naubot.activarFoundation(true);
        sleep(500);
        naubot.moverDistanciaRecta(70);
        runtime.reset();
        double periodo = runtime.milliseconds() + 3000;
        while(periodo > runtime.milliseconds()) {
            naubot.leftDrive.setPower(1);
            naubot.rightDrive.setPower(-1);
        }
    }
}
