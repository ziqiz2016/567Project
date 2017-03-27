#define L1 0.5
#define L2 4
#define L3 4.5
#define k_cl 30*0.02
#define N 50

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


