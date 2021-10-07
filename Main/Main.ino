#include <MatrixMini.h>

#include <FABRIK2D.h>

#include "Configuration.h"
#include "Definitions.h"

void setup()
{
    Serial.begin(115200); // set up Serial library at 115200 bps
    
    Serial.println("Matrix Mini Test - PIXY2 Camera\n");
    Serial.println("PIXY2 Camera on I2C port 1");

    Mini.begin();

    // Servo setup
    Mini.RC1.set(0);
    Mini.RC2.set(0);
    // sigmap means allow signature range
    // Ex: sigmap = 3, means allowed signatures are: 1, 2, 3. (max is 7)
    Mini.I2C1.PIXYcam.pixyinit(3);
    cameraProcessInit();
}

int dt = 16; //60 Hz = 16.66ms

void loop()
{
    // Get specific signature, with n_th largest block.
    // Note: getblock(signature, n_th) should be use before other functions.
    // getblock(0, 1) means select the largest block with no specific signature.
    
    bool isBlockValid = Mini.I2C1.PIXYcam.getblock(1, 1);
    if (isBlockValid == true)
    {
        // Serial.print("Count of blocks : ");
        // Serial.println(Mini.I2C1.PIXYcam.getCountofBlock());
        
        cameraProcess(
            Mini.I2C1.PIXYcam.block.x + Mini.I2C1.PIXYcam.block.width/2, 
            Mini.I2C1.PIXYcam.block.y + Mini.I2C1.PIXYcam.block.height/2,
            dt);
        delay(100);
        Serial.print("puck_x = ");
        Serial.print(puckCoordX);
        Serial.print(", puck_y = ");
        Serial.println(puckCoordY);

        Serial.println("=====================");
    }
}
