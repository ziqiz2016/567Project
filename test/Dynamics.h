#define L1 0
#define L2 0
#define L3 0
#define k_cl 60
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
    float px,py,pz;    // coordinate
    float J[3][3];     // Jacobian
public:
    float th1,th2,th3; // joint angle
    ThreeLinks(float,float,float);
    ~ThreeLinks();
    void Jacobian();
    void update(Path *path_des, int i);
};


