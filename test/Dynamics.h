#define L1 3.9//4.3
#define L2 9.4
#define L3 10.5
#define off1 2.8
#define off2 -3.7
#define k_cl 40*0.01
#define N 100

class Path
{
 public:
    float x[N];
    float y[N];
    float z;
};

class ThreeLinks
{
    float J[3][3];     // Jacobian
public:
    float px,py,pz;    // coordinate
    float th1,th2,th3; // joint angle
    ThreeLinks(float,float,float);
    ~ThreeLinks();
    void Jacobian();
    void update(Path *path_des, int i);
};
