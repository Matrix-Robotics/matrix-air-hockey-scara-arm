#include <MatrixMini.h>
#include <FABRIK2D.h>

#include "Configuration.h"
#include "Definitions.h"

int lengths[] = {192, 216}; // {shoulder, elbow} link length
float initAngle[] = {101, -162};

Fabrik2D fabrik2D(3, lengths); // 3 Joints in total

void setup()
{
    Serial.begin(115200); // set up Serial library at 115200 bps

    Serial.println("Matrix Mini Test - PIXY2 Camera\n");
    Serial.println("PIXY2 Camera on I2C port 1");

    Mini.begin();

    // Servo setup
    Mini.RC1.set(0);
    Mini.RC2.set(0);

    // Pixy setup
    // sigmap means allow signature range
    // Ex: sigmap = 3, means allowed signatures are: 1, 2, 3. (max is 7)
    Mini.I2C1.PIXYcam.pixyinit(3);
    cameraProcessInit();

    // IK solver setup
    fabrik2D.setTolerance(0.5);
    // fabrik2D.setJoints(initAngle, lengths);

    // Serial.print("ang0");
    // Serial.print("\t");
    // Serial.print("ang1");
    // Serial.print("\t");
    // Serial.print("x0");
    // Serial.print("\t");
    // Serial.print("y0");
    // Serial.print("\t");
    // Serial.print("x1");
    // Serial.print("\t");
    // Serial.print("y1");
    // Serial.print("\t");
    // Serial.print("x2");
    // Serial.print("\t");
    // Serial.println("y2");
}

int dt = 16; //60 Hz = 16.66ms
int pos = 0; // variable to store the servo position

int shoulderAngle;
int elbowAngle;


float x = 0;
int toggle_x = 0;


void loop()
{
    // Move from -30 to 40 in the y axis
    if (x < 100) {
        toggle_x = 0;
        x = 100;
    } else if (x > 250) {
        toggle_x = 1;
        x = 250;
    }

    // Solve inverse kinematics given the coordinates x and y and the list of lengths for the arm.
    fabrik2D.solve(x, 0, lengths);


    bool isBlockValid = Mini.I2C1.PIXYcam.getblock(1, 1);
    if (isBlockValid == true)
    {
        cameraProcess(
            Mini.I2C1.PIXYcam.block.x + Mini.I2C1.PIXYcam.block.width / 2,
            Mini.I2C1.PIXYcam.block.y + Mini.I2C1.PIXYcam.block.height / 2,
            dt);
        robotDetection(
            Mini.I2C1.PIXYcam.block.x + Mini.I2C1.PIXYcam.block.width / 2,
            Mini.I2C1.PIXYcam.block.y + Mini.I2C1.PIXYcam.block.height / 2);
        

        // Serial.print("puck_x = ");
        // Serial.print(puckCoordX);
        // Serial.print(", puck_y = ");
        // Serial.println(puckCoordY);
        // Serial.println("=====================");

        int shoulderAngle = fabrik2D.getAngle(0) * RAD_TO_DEG; // In degrees
        int elbowAngle = fabrik2D.getAngle(1) * RAD_TO_DEG;    // In degrees

        // Serial.print("shoulderAngle = ");
        // Serial.print(shoulderAngle);
        // Serial.print(", elbowAngle = ");
        // Serial.println(elbowAngle);
        // Serial.println("=====================");
        Mini.RC1.set(140 - shoulderAngle);
        Mini.RC2.set(160 + elbowAngle);
    }

    if (toggle_x == 0) {
        x=x+20;
    } else {
        x=x-20;
    }


    // ============ Processing simulator ============
    // Serial.print(fabrik2D.getAngle(0) * 57296 / 1000);
    // Serial.print("\t");
    // Serial.print(fabrik2D.getAngle(1) * 57296 / 1000);
    // Serial.print("\t");
    // Serial.print(fabrik2D.getX(0));
    // Serial.print("\t");
    // Serial.print(fabrik2D.getY(0));
    // Serial.print("\t");
    // Serial.print(fabrik2D.getX(1));
    // Serial.print("\t");
    // Serial.print(fabrik2D.getY(1));
    // Serial.print("\t");
    // Serial.print(fabrik2D.getX(2));
    // Serial.print("\t");
    // Serial.println(fabrik2D.getY(2));
}
