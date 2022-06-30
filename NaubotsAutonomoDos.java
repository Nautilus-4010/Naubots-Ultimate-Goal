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

@Autonomous(name="Stones Red Alliance", group="Linear Opmode")

public class NaubotsAutonomoDos extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private LaBarca naubot = new LaBarca(this);

    @Override
    public void runOpMode() {

        naubot.getHardware(hardwareMap);
        naubot.resetEncoders();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        //Primer Stone
        naubot.moverDistanciaRecta(60);
        runtime.reset();
        double periodo = runtime.milliseconds() + 2000;
        while(!naubot.boton.isPressed() && runtime.milliseconds() < periodo && opModeIsActive()) {
            naubot.activarIntake(-1);
            naubot.leftDrive.setPower(0.3);
            naubot.rightDrive.setPower(0.3);
        }
        naubot.moverDistanciaRecta(-70);
        naubot.girarEnEje(-45);
        naubot.moverDistanciaRecta(150);
        naubot.activarIntake(1);
        sleep(1000);
        naubot.moverDistanciaRecta(-220);
        naubot.girarEnEje(50);

        //Comenzar por el segundo Stone
        periodo = runtime.milliseconds() + 2000;
        while(!naubot.boton.isPressed() && runtime.milliseconds() < periodo && opModeIsActive()) {
            naubot.activarIntake(-1);
            naubot.leftDrive.setPower(0.3);
            naubot.rightDrive.setPower(0.3);
        }
        naubot.moverDistanciaRecta(-70);
        naubot.girarEnEje(-50);
        naubot.moverDistanciaRecta(170);
        naubot.activarIntake(1);
        sleep(1000);
        naubot.moverDistanciaRecta(-220);
        naubot.girarEnEje(50);

        //Comenzar por el tercer Stone
        periodo = runtime.milliseconds() + 2000;
        while(!naubot.boton.isPressed() && runtime.milliseconds() < periodo && opModeIsActive()) {
            naubot.activarIntake(-1);
            naubot.leftDrive.setPower(0.3);
            naubot.rightDrive.setPower(0.3);
        }
        naubot.moverDistanciaRecta(-70);
        naubot.girarEnEje(-50);
        naubot.moverDistanciaRecta(170);
        naubot.activarIntake(1);
        sleep(1000);
    }
}
