/*Copyright 2019

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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Pinguinita", group="Linear Opmode")
public class Pinguinita extends LinearOpMode {
  private ElapsedTime runtime = new ElapsedTime();
  private LaBarca naubot = new LaBarca(this);

  @Override
  public void runOpMode() {

    naubot.getHardware(hardwareMap);
    naubot.resetEncoders();

    telemetry.addData("Status", "Initialized");
    telemetry.update();
    waitForStart();

    runtime.reset();
    boolean click = false;
    boolean click1 = false;
    boolean click2 = false;
    boolean click3 = false;
    boolean click4 = false;
    boolean click5 = false;
    boolean modoDriver = false;
    boolean foundation = false;

    while (opModeIsActive()) {
      double leftPower, rightPower, intakePower;

      if(gamepad1.back){
        click = true;
      } else if ( !gamepad1.back && click){
        modoDriver = !modoDriver;
        click = false;
      }

      if(modoDriver){
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        leftPower    = Range.clip(drive + turn, -1.0, 1.0);
        rightPower   = Range.clip(drive - turn, -1.0, 1.0);
      } else {
        leftPower = -gamepad1.left_stick_y;
        rightPower = -gamepad1.right_stick_y;
      }

      if(gamepad1.left_bumper){
        leftPower *= 0.5;
        rightPower *= 0.5;
      } else if(!gamepad1.right_bumper){
        leftPower *= 0.75;
        rightPower *= 0.75;
      }

      if(gamepad2.right_trigger > 0){
        intakePower = -1;
      } else if(gamepad2.left_trigger > 0){
        intakePower = 1;
      } else {
        intakePower = 0;
      }

      double elevadorPower;
      if(gamepad2.dpad_up) {
        elevadorPower = 1;
      } else if(gamepad2.dpad_down) {
        elevadorPower = -1;
      } else {
        elevadorPower = 0;
      }

      if(gamepad1.b){
        click3 = true;
      } else if ( !gamepad1.b && click3){
        foundation = !foundation;
        click3 = false;
      }

      if(gamepad1.left_trigger > 0 && !click1){
        naubot.setGiroDeNoventaGrados(-1);
        click1 = true;
      } else if(gamepad1.left_trigger == 0 && click1){
        click1 = false;
      }

      if(gamepad1.right_trigger > 0 && !click2){
        naubot.setGiroDeNoventaGrados(1);
        click2 = true;
      } else if(gamepad1.right_trigger == 0 && click2){
        click2 = false;
      }

      if(naubot.leftDrive.isBusy() && naubot.rightDrive.isBusy()) {
        leftPower = 1;
        rightPower = 1;
      } else
        naubot.defaultRunmode();

      double extensionPower = 0;
      if(gamepad2.y)
        extensionPower = 1;
      else if(gamepad2.a)
        extensionPower = -1;

      naubot.leftDrive.setPower(leftPower);
      naubot.rightDrive.setPower(rightPower);
      naubot.activarElevador(elevadorPower);
      naubot.activarFoundation(foundation);
      naubot.activarExtension(extensionPower);

      if(gamepad2.right_bumper || gamepad2.left_bumper)
        naubot.activarIntake();
      else
        naubot.activarIntake(intakePower);

      telemetry.addData("Status", "Run Time: " + runtime.toString());
      telemetry.addData("Modo conduccion:", modoDriver ? "POV" : "Tanque");
      telemetry.addData("Elevador: ", naubot.posicionElevador());
      //telemetry.addData("Velocidad motor izquierdo:", leftPower);
      //telemetry.addData("Velocidad motor derecho:", rightPower);
      //telemetry.addData("Boton: ", naubot.boton.isPressed());
      telemetry.addData("Encoders: ", naubot.leftDrive.getCurrentPosition() + ", " + naubot.rightDrive.getCurrentPosition());
      telemetry.update();
    }
  }
}
