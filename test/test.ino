#include <Servo.h>
#include "Dynamics.h"
#define pi 3.14
#define r2d 180/pi  // radius to degree
#define d2r pi/180
#define r  3        // radius of circle
#define p  6
#define T  200      // t_Step 200ms


Servo servo1, servo2, servo3;
int q1, q2, q3;         // offset of the coordinate
int state;
float x, y;             // coordinate for start point
float th1, th2, th3;

void setup() {
  // Opens serial port, sets data rate to 9600 bps
  Serial.begin(9600);
  //initial_angle();        // initialize angle
  // attaches the servo on pin 9 to the servo object
  servo1.attach(9);       // #9 for q1
  servo2.attach(10);      // #10 for q2
  servo3.attach(11);      // #11 for q3
  q1 = 35;
  q2 = 52-3;
  q3 = 84;
  th1 = 0;
  th2 = -pi / 2;
  th3 = -pi / 2;
  state = 4;
  servo1.write(q1);
  servo2.write(q2);
  servo3.write(q3);
  Serial.println("***********************Drawing Robot***********************");
  Serial.println("Please press x to set a start point on the drawing sheet.");
  Serial.println("***********************************************************");
  //servo1.write(0);
}

void loop() {
  //Serial.println((mylinks.th1)*r2d);
  //delay(15);  // let the arm go to the initial position
  // send data only when you receive data:
  switch (state) {
    case 1: set_angle1(); break;
    case 2: set_angle2(); break;
    case 3: set_angle3(); break;
    case 4: work_mode(); break;
    default: set_angle1(); break;
  }
}

void work_mode() {
  char incomingData;      // for incoming serial data
  Path path;
  ThreeLinks mylinks(th1, th2, th3);
  mylinks.Jacobian();
  if (Serial.available() > 0) {


    // read the incoming byte:
    incomingData = Serial.read();
    // say what you got:
    Serial.print("Message Received: ");
    Serial.println(incomingData);
    switch (incomingData)
    {
      case 's': draw_square(&mylinks, &path, x, y); break;
      case 'c': draw_circle(&mylinks, &path, x, y); break;
      case 't': draw_tri(&mylinks, &path, x, y); break;
      case 'x': start_point(&mylinks, &x, &y); break;
      default : Serial.println("nothing to draw"); break;
    }
    //Serial.print("\r"); //enter
  }
}

// initialize angle
void set_angle1()
{
  int tmp;
  Serial.print("set first joint angle:");
  while (!(Serial.available() > 0)) {}
  tmp = Serial.parseInt();
  q1 += tmp;
  Serial.println(q1, DEC);
  servo1.write(q1);
  if (tmp == 0)
  {
    state = 2;
  }
}
void set_angle2()
{
  int tmp;
  Serial.print("set second joint angle:");
  while (!(Serial.available() > 0)) {}
  tmp = Serial.parseInt();
  q2 += tmp;
  Serial.println(q2, DEC);
  servo2.write(q2);
  if (tmp == 0)
  {
    state = 3;
  }
}
void set_angle3()
{
  int tmp;
  Serial.print("set third joint angle:");
  while (!(Serial.available() > 0)) {}
  tmp = Serial.parseInt();
  q3 += tmp;
  Serial.println(q3, DEC);
  servo3.write(q3);
  if (tmp == 0)
  {
    state = 4;
    // Print out instructions

  }
}
// output torque
void output(const ThreeLinks *mylinks)
{
  servo1.write(round(r2d * (mylinks->th1)) + q1);
  servo2.write(round(r2d * (mylinks->th2)) + q2 + 90);
  servo3.write(round(r2d * (mylinks->th3)) + q3 + 90);
  //Serial.print(mylinks->px, DEC);
  //Serial.print(round(r2d * (mylinks->th1)), DEC);
  //Serial.print(' ');
  //Serial.print(mylinks->py, DEC);
  //Serial.print(round(r2d * (mylinks->th2)), DEC);
  //Serial.print(' ');
  //Serial.println(mylinks->pz, DEC);
  //Serial.println(round(r2d * (mylinks->th3)), DEC);
}

// initialize path
void initial_path(Path *path, float x0, float y0,char shape)
{
  float z0 = 0;
  switch (shape) {
  case 'c':
    for (int i = 0; i < N; ++i)
    {
      path->x[i] = (x0 - r) + r * cos(i * 2 * pi / N);
      path->y[i] = y0 + r * sin(i * 2 * pi / N);
    }
    path->z = z0;
  break;
  case 't':
    for (int i = 0; i < N; ++i)
    {
      if(i<30)
      {
          path->x[i]=x0+p*(i-0)/30;
          path->y[i]=-path->x[i]+x0+y0;
      }
      else if(i<70)
      {
          path->x[i]=x0+p-2*p*(i-30)/40;
          path->y[i]=y0-p;
      }
      else
      {
          path->x[i]=x0-p+p*(i-70)/30;
          path->y[i]=path->x[i]-x0+y0;
      }
    }
    path->z = z0;
  break;
  case 's':
    for (int i = 0; i < N; ++i)
    {
      if(i<25)
      {
          path->x[i]=x0-p*(i-0)/25;
          path->y[i]=y0;
      }
      else if(i<50)
      {
          path->x[i]=x0-p;
          path->y[i]=y0-p*(i-25)/25;
      }
      else if(i<75)
      {
          path->x[i]=x0-p+p*(i-50)/25;
          path->y[i]=y0-p;
      }
      else
      {
          path->x[i]=x0;
          path->y[i]=y0-p+p*(i-75)/25;
      }
    }
    path->z = z0;
  break;
  case 'p':    // point
    for (int i = 0; i < N; ++i)
    {
      path->x[i] = x0;
      path->y[i] = y0;
    }
    path->z = z0;
  break;
}
  // for (int i = 0; i < N; ++i)
  // {
  //   path->x[i] = (x0 - r) + r * cos(i * 2 * pi / N);
  //   path->y[i] = y0 + r * sin(i * 2 * pi / N);
  //   Serial.print(path->x[i], DEC);
  //   Serial.print(" ");
  //   Serial.println(path->y[i], DEC);
  // }
  // path->z = z0;
  Serial.println("path initial completed.");
}

// set start point
void start_point(ThreeLinks *mylinks, float *x, float *y)
{
  Path point;
  Serial.print("set x:");           //x=11 y=6
  while (!(Serial.available() > 0)) {}
  *x = Serial.parseFloat();
  Serial.println(*x, DEC);
  Serial.print("set y:");
  while (!(Serial.available() > 0)) {}
  *y = Serial.parseFloat();
  Serial.println(*y, DEC);
  point.x[0] = *x;
  point.y[0] = *y;
  point.z = 0.1;         // cal_point coordinate
  //Serial.println("Calibration......");
  for (int i = 0; i < 20; ++i)
  {
    output(mylinks);
    mylinks->update(&point, 0);
    th1 = mylinks->th1;
    th2 = mylinks->th2;
    th3 = mylinks->th3;
    //output(mylinks);
    delay(T); // delay some time
  }
  Serial.println("Finished Calibration......");
  Serial.println("***************Drawing Robot******************");
  Serial.println("  Press the following key to draw a shape.");
  Serial.println("  x: setting another start point          ");
  Serial.println("  c: circle                               ");
  Serial.println("  t: triangle                             ");
  Serial.println("  s: square                               ");
  Serial.println("**********************************************");
}

// draw circle
void draw_circle(ThreeLinks *mylinks, Path *circle, float x, float y)
{
  initial_path(circle, x, y,'c');
  Serial.println("I'm drawing a circle......");
  for (int i = 0; i < N; ++i)
  {
    mylinks->update(circle, i);
    output(mylinks);
    delay(T); // delay some time
  }
  for (int j=0;j<10;++j)
  {
    mylinks->update(circle, 0);
    output(mylinks);
  }
  Serial.println("circle draw completed.");
  Serial.println("***************Drawing Robot******************");
  Serial.println("  Press the following key to draw a shape.");
  Serial.println("  x: setting another start point          ");
  Serial.println("  c: circle                               ");
  Serial.println("  t: triangle                             ");
  Serial.println("  s: square                               ");
  Serial.println("**********************************************");
}
void draw_square(ThreeLinks *mylinks, Path *square, float x, float y)
{
  initial_path(square, x, y,'s');
  Serial.println("I'm drawing a square......");
  for (int i = 0; i < N; ++i)
  {
    mylinks->update(square, i);
    output(mylinks);
    delay(T); // delay some time
  }
  for (int j=0;j<10;++j)
  {
    mylinks->update(square, 0);
    output(mylinks);
  }
  Serial.println("square draw completed.");
  Serial.println("***************Drawing Robot******************");
  Serial.println("  Press the following key to draw a shape.");
  Serial.println("  x: setting another start point          ");
  Serial.println("  c: circle                               ");
  Serial.println("  t: triangle                             ");
  Serial.println("  s: square                               ");
  Serial.println("**********************************************");
}
void draw_tri(ThreeLinks *mylinks, Path *triangle, float x, float y)
{
  initial_path(triangle, x, y,'t');
  Serial.println("I'm drawing a triangle......");
  for (int i = 0; i < N; ++i)
  {
    mylinks->update(triangle, i);
    output(mylinks);
    delay(T); // delay some time
  }
  for (int j=0;j<10;++j)
  {
    mylinks->update(triangle, 0);
    output(mylinks);
  }
  Serial.println("triangle draw completed.");
  Serial.println("***************Drawing Robot******************");
  Serial.println("  Press the following key to draw a shape.");
  Serial.println("  x: setting another start point          ");
  Serial.println("  c: circle                               ");
  Serial.println("  t: triangle                             ");
  Serial.println("  s: square                               ");
  Serial.println("**********************************************");
}
