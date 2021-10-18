#include <MatrixMini.h>
#include <FABRIK2D.h>

#include "Configuration.h"
#include "Definitions.h"

int lengths[] = {192, 216}; // {shoulder, elbow} link length
float initAngle[] = {101, -162};

int dt = 16; //60 Hz = 16.66ms

Fabrik2D fabrik2D(3, lengths); // 3 Joints in total

void setup()
{
    Serial.begin(115200); // set up Serial library at 115200 bps

    Serial.println("Matrix Mini Test - PIXY2 Camera\n");
    Serial.println("PIXY2 Camera on I2C port 1");

    Mini.begin();

    // Servo setup
    shoulder_old_angle = 50;
    elbow_old_angle = 7;
    Mini.RC1.set(0);
    Mini.RC2.set(0);

    // Pixy setup
    // sigmap means allow signature range
    // Ex: sigmap = 3, means allowed signatures are: 1, 2, 3. (max is 7)
    Mini.I2C1.PIXYcam.pixyinit(1);
    cameraProcessInit();

    // IK solver setup
    fabrik2D.setTolerance(1);
    // fabrik2D.setJoints(initAngle, lengths);

    // Robot strategy setup
    defense_position = ROBOT_DEFENSE_POSITION;
    attack_position = ROBOT_DEFENSE_ATTACK_POSITION;

    // Timer setup
    timer_old = micros();
    servo_timer_old = micros();
}

void loop()
{
    timer_value = micros();

    if ((timer_value - timer_old) >= 1000) // 1Khz loop
    {
        timer_old = timer_value;
        bool isBlockValid = Mini.I2C1.PIXYcam.getblock(0, 1);

        if (isBlockValid == true)
        {
            cameraProcess(
                Mini.I2C1.PIXYcam.block.x + Mini.I2C1.PIXYcam.block.width / 2,
                Mini.I2C1.PIXYcam.block.y + Mini.I2C1.PIXYcam.block.height / 2,
                dt);

            newDataStrategy();

            // Serial.print("puck_x = ");
            // Serial.print(puckCoordX);
            // Serial.print(", puck_y = ");
            // Serial.println(puckCoordY);
            // Serial.println("=====================");
        }

        robotStrategy();

        // Serial.print("robot_status: ");
        // Serial.println(robot_status);

        // Solve inverse kinematics given the coordinates x and y and the list of lengths for the arm.
        // fabrik2D.solve(100, y, lengths);
        fabrik2D.solve(com_pos_x, com_pos_y, lengths);
        // fabrik2D.solve(260, -210, lengths);

        shoulder_solver_angle = 140 - fabrik2D.getAngle(0) * RAD_TO_DEG; // In degrees
        elbow_solver_angle = 160 + fabrik2D.getAngle(1) * RAD_TO_DEG;    // In degrees

        // Serial.print("com_pos_x = ");
        // Serial.print(com_pos_x);
        // Serial.print(", com_pos_y = ");
        // Serial.println(com_pos_y);
        // Serial.println("=====================");

        // Serial.print("fabrik2D.getAngle(0) = ");
        // Serial.print(fabrik2D.getAngle(0));
        // Serial.print(", elbow_angle = ");
        // Serial.println(fabrik2D.getAngle(1));

        Serial.print("shoulder_solver_angle = ");
        Serial.print(shoulder_solver_angle);
        Serial.print(", elbow_solver_angle = ");
        Serial.println(elbow_solver_angle);
        Serial.println("=====================");
    }


    if (!to_move)
    {
        if (shoulder_solver_angle != shoulder_old_angle || elbow_solver_angle != elbow_old_angle)
        {
            to_move = true;

            shoulder_angle = shoulder_solver_angle;
            elbow_angle = elbow_solver_angle;

            servo_timer_old = timer_value;
        } else {
            Mini.RC1.set(shoulder_solver_angle);
            Mini.RC2.set(elbow_solver_angle);
        }
    }
    
    servo_progress = timer_value - servo_timer_old;

    if (servo_progress <= servo_move_time)
    {
        shoulder_map_angle = map(servo_progress, 0, servo_move_time, shoulder_old_angle, shoulder_angle);
        elbow_map_angle = map(servo_progress, 0, servo_move_time, elbow_old_angle, elbow_angle);
        
        // Servo control
        Mini.RC1.set(shoulder_map_angle);
        Mini.RC2.set(elbow_map_angle);
        // Serial.print("shoulder_angle = ");
        // Serial.print(shoulder_map_angle);
        // Serial.print(", elbow_angle = ");
        // Serial.println(elbow_map_angle);
        // Serial.println("=====================");
        // Serial.print("servo_progress = ");
        // Serial.println(servo_progress);
        // Serial.println("=====================");
    }
    else
    {
        to_move = false;
        shoulder_old_angle = shoulder_angle;
        elbow_old_angle = elbow_angle;

        servo_timer_old = timer_value;
    }

    // Serial.print("shoulder_angle = ");
    // Serial.print(shoulder_map_angle);
    // Serial.print(", elbow_angle = ");
    // Serial.println(elbow_map_angle);
    // Serial.println("=====================");
    
}
