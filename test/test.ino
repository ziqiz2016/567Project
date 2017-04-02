#include <Servo.h>
#include "Dynamics.h"
#define pi 3.14
#define r2d 180/pi  // radius to degree
#define d2r pi/180
#define r  1        // radius of circle
#define T  200      // t_Step 200ms


Servo servo1, servo2, servo3;
float q1,q2,q3;

void setup() {
    // Opens serial port, sets data rate to 9600 bps
    Serial.begin(9600);
    initial_angle();        // initialize angle
    // attaches the servo on pin 9 to the servo object
    servo1.attach(9);       // #9 for q1
    servo2.attach(10);      // #10 for q2
    servo3.attach(11);      // #11 for q3
}

void loop() {
    int i;
    float x,y;              // coordinate for start point
    char incomingData;      // for incoming serial data
    Path path;
    ThreeLinks mylinks(q1,q2,q3);
    mylinks.Jacobian();
    //Serial.println((mylinks.th1)*r2d);
    //output(&mylinks);
    //delay(15);  // let the arm go to the initial position
    // send data only when you receive data:
    if (Serial.available() > 0){


        // read the incoming byte:
        incomingData = Serial.read();
        // say what you got:
        Serial.print("Message Received: ");
        Serial.println(incomingData);
        switch(incomingData)
        {
            case 's': start_point(&mylinks,&x,&y); break;
            case 'c': draw_circle(&mylinks,&path,x,y); break;
            default : Serial.println("nothing to draw."); break;
        }
        //Serial.print("\r"); //enter
    }
}



// output torque
void output(const ThreeLinks *mylinks)
{
    servo1.write(r2d*(mylinks->th1));
    Serial.println(r2d*(mylinks->th1), DEC);
    servo2.write(r2d*(mylinks->th2));
    servo3.write(r2d*(mylinks->th3));
}

// initialize path
void initial_circle(Path *path, float x0, float y0)
{
    float z0=0.5;
    for(int i=0;i<N;++i)
    {
        path->x[i]= (x0-r) + r*cos(i*2*pi/N);
        path->y[i]= y0 + r*sin(i*2*pi/N);
        Serial.print(path->x[i], DEC);
        Serial.print(" ");
        Serial.println(path->y[i], DEC);
    }
    path->z= z0;
    Serial.println("circle initial completed.");
}

// initialize angle
void initial_angle()
{
  Serial.print("set first joint angle:");
  while(!(Serial.available() > 0)){}
  q1 = Serial.parseFloat()*d2r;
  Serial.println(q1, DEC);
  Serial.print("set second joint angle:");
  while(!(Serial.available() > 0)){}
  q2 = Serial.parseFloat()*d2r;
  Serial.println(q2, DEC);
  Serial.print("set third joint angle:");
  while(!(Serial.available() > 0)){}
  q3 = Serial.parseFloat()*d2r;
  Serial.println(q3, DEC);
  Serial.println("finished initializing angle.");
}

// set start point
void start_point(ThreeLinks *mylinks,float *x,float *y)
{
  Path point;
  Serial.print("set x:");
  while(!(Serial.available() > 0)){}
  *x = Serial.parseFloat();
  Serial.println(*x, DEC);
  Serial.print("set y:");
  while(!(Serial.available() > 0)){}
  *y = Serial.parseFloat();
  Serial.println(*y, DEC);
  point.x[0]=*x;
  point.y[0]=*y;
  point.z=0;           // cal_point coordinate
  //Serial.println("Calibration......");
  for (int i = 0; i < 5; ++i)
  {
      mylinks->update(&point,0);
      output(mylinks);
      delay(T); // delay some time
  }
  //Serial.println("Finished Calibration......");
}

// draw circle
void draw_circle(ThreeLinks *mylinks,Path *circle,float x,float y)
{
    initial_circle(circle,x,y);
    Serial.println("I'm drawing a circle......");
    for (int i = 0; i < N; ++i)
    {
        mylinks->update(circle,i);
        output(mylinks);
        delay(T); // delay some time
    }
    Serial.println("circle draw completed.");
}
