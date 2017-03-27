#include <Servo.h>
#include "Dynamics.h"
#define pi 3.14
#define r  1
    
Path circle;
Servo servo1, servo2, servo3;

void setup() {
    // Opens serial port, sets data rate to 9600 bps
    Serial.begin(9600);
    initial(&circle);  // initialize the Path
    // attaches the servo on pin 9 to the servo object 
    servo1.attach(9);
    servo1.attach(10);
    servo1.attach(11); // need to correct to the right pin
}

void loop() {
    int i;
    char incomingData;      // for incoming serial data
    ThreeLinks mylinks(0,pi/6,pi/3);
    output(&mylinks);
    delay(15);  // let the arm go to the initial position
    // send data only when you receive data:
    if (Serial.available() > 0){
        
            
        // read the incoming byte:
        incomingData = Serial.read();
        // say what you got:
        Serial.print("Message Received: ");
        Serial.println(incomingData);
        switch(incomingData)
        {
            case 'c': draw_circle(&mylinks); break;
            default : Serial.println("nothing to draw."); break;
        }
        //Serial.print("\r"); //enter
    }
}



// output torque
void output(const ThreeLinks *mylinks)
{
    servo1.write(mylinks->th1);
    servo2.write(mylinks->th2);
    servo3.write(mylinks->th3);
}

// initialize path
void initial(Path *path)
{
    float x0=2;
    float y0=2;
    float z0=0;
    int i;
    for(i=0;i<N;++i)
    {
        path->x[i]= x0 + cos(i*2*pi/N);
        path->y[i]= y0 + sin(i*2*pi/N);
    }
    path->z= z0;
    Serial.println("circle initial completed.");
}

// draw circle
void draw_circle(ThreeLinks *mylinks)
{
    Serial.println("I'm drawing a circle......");
    for (int i = 0; i < N; ++i)
    {
        mylinks->update(&circle,i);
        output(mylinks);
        delay(2); // delay some time 
    }
    Serial.println("circle draw completed.");
}
