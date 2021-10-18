// AHR AIR HOCKEY ROBOT PROJECT

// void setPosition()
// {

// }

// Each time a new data packet from camera is reveived this function is called
void newDataStrategy()
{
    // predict_status == 1 => Puck is moving to our field directly
    // predict_status == 2 => Puck is moving to our field with a bounce
    // predict_status == 3 => Puck is in our field moving slowly, attack?

    // Default
    robot_status = 0; // Going to initial position (defense)

    if ((predict_status == 1) && (predict_time < 350))
    {
        // WE come from a bounce?
        if (predict_bounce == 1)
        {
            if ((puckSpeedXAverage) > -150)
                // puck is moving slowly...
                robot_status = 2; // Defense+Attack
            else
            {
                if (predict_y < -200)
                    predict_y = -200;
                if (predict_y > 200)
                    predict_y = 200;
                robot_status = 4;
            }
        }
        else
        {
            if ((predict_y > (ROBOT_MIN_Y + 35)) && (predict_y < (ROBOT_MAX_Y - 35)))
                robot_status = 2; //2  //Defense+attack mode
            else
                robot_status = 1; //1
        }
    }

    // Prediction with side bound
    if ((predict_status == 2) && (predict_time < 350))
    {
        robot_status = 1; //1   // Defense mode
        // We limit the movement in this phase
        if (predict_y < -200)
            predict_y = -200;
        if (predict_y > 200)
            predict_y = 200;
    }

    // If the puck is moving slowly in the robot field we could start an attack
    if ((predict_status == 0) && (puckCoordX < ROBOT_CENTER_X) && (myAbs(puckSpeedX) < 50))
        robot_status = 3; //3
}

void robotStrategy()
{
    switch (robot_status)
    {
    case 0:
        // Go to defense position
        com_pos_x = defense_position;
        com_pos_y = ROBOT_CENTER_Y; //center Y axis
        // setSpeedS(com_speed_y, com_speed_x);
        // setPosition(com_pos_x, com_pos_y);
        attack_time = 0;
        break;
    case 1:
        // Defense mode (only move on X axis on the defense line)
        com_pos_x = defense_position;
        com_pos_y = predict_y;
        // setSpeedS(com_speed_y, com_speed_x);
        // setPosition(com_pos_x, com_pos_y);
        attack_time = 0;
        break;
    case 2:
        // Defense+attack
        if (predict_time_attack < 180) // If time is less than 180ms we start the attack
        {
            com_pos_x = attack_position + 60;
            com_pos_y = predict_y_attack;
            // setSpeedS(com_speed_y, com_speed_x);
            // setPosition(com_pos_x, com_pos_y); // We use a straight line path
        }
        else // Defense position
        {
            com_pos_x = predict_x;
            com_pos_y = predict_y; // predict_y_attack;
            // setSpeedS(com_speed_y, com_speed_x);
            // setPosition(com_pos_x, com_pos_y);
            attack_time = 0;
        }

        break;
    case 3:
        // ATTACK MODE
        if (attack_time == 0)
        {
            attack_predict_y = predictPuckYPosition(200);
            attack_predict_x = predictPuckXPosition(200);
            if ((attack_predict_y > -210) && (attack_predict_y < 210) && (attack_predict_x > 60) && (attack_predict_x < 280))
            {
                attack_time = millis() + 300;    // Prepare an attack in 500ms
                attack_pos_y = attack_predict_y; // predict_x
                attack_pos_x = attack_predict_x; // predict_y
                Serial.print("AM:");
                //Serial.print(attack_time);
                //Serial.print(",");
                Serial.print(attack_pos_y);
                Serial.print(",");
                Serial.println(attack_pos_x);
                //Serial.print(" ");
                // Go to pre-attack position
                com_pos_y = attack_pos_y;
                com_pos_x = attack_pos_x - 60;
                // setSpeedS(com_speed_y / 2, com_speed_x / 2);
                // setPosition(com_pos_x, com_pos_y);
                attack_status = 1;
            }
            else
            {
                attack_time = 0; // Continue...
                attack_status = 0;
                // And go to defense position
                com_pos_x = defense_position;
                com_pos_y = ROBOT_CENTER_Y; //center Y axis
                // setSpeedS(com_speed_y / 2, com_speed_x / 2);
                // setPosition(com_pos_x, com_pos_y);
            }
        }
        else
        {
            if (attack_status == 1)
            {
                if ((attack_time - millis()) < 300)
                {
                    // Attack movement
                    com_pos_y = predictPuckYPosition(50);
                    com_pos_x = predictPuckXPosition(50) + 50;
                    // setSpeedS(com_speed_y, com_speed_x);
                    // setPosition(com_pos_x, com_pos_y);

                    Serial.print("ATTACK:");
                    Serial.print(com_pos_y);
                    Serial.print(",");
                    Serial.println(com_pos_x - 50);

                    attack_status = 2; // Attacking
                }
                else // attack_status=1 but it´s no time to attack yet
                {
                    // Go to pre-attack position
                    com_pos_y = attack_pos_y;
                    com_pos_x = attack_pos_x - 60;
                    // setSpeedS(com_speed_y / 2, com_speed_x / 2);
                    // setPosition(com_pos_x, com_pos_y);
                }
            }
            if (attack_status == 2)
            {
                if (millis() > (attack_time + 50)) // Attack move is done? => Reset to defense position
                {
                    Serial.print("RESET");
                    attack_time = 0;
                    robot_status = 0;
                    attack_status = 0;
                }
            }
        }
        break;
    case 4:
        // The puck came from a bounce
        // Only defense now (we could improve this in future)
        // Defense mode (only move on X axis on the defense line)
        com_pos_x = defense_position;
        com_pos_y = predict_y;
        // setSpeedS(com_speed_y, com_speed_x);
        // setPosition(com_pos_x, com_pos_y);
        attack_time = 0;
        break;
    default:
        // Default : go to defense position
        com_pos_x = defense_position;
        com_pos_y = ROBOT_CENTER_Y; // center
        // setSpeedS(com_speed_y, com_speed_x);
        // setPosition(com_pos_x, com_pos_y);
        attack_time = 0;
    }
}
