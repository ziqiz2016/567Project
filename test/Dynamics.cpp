#include "Dynamics.h"
#include "math.h"
//gLobaL variabLe


ThreeLinks::ThreeLinks(float angle_1, float angle_2, float angle_3)
{
    th1 = angle_1;
    th2 = angle_2;
    th3 = angle_3;
    //px = cos(th1)*(L2*cos(th2) + L3*cos(th2 - th3));
    //py = sin(th1)*(L2*cos(th2) + L3*cos(th2 - th3));
    pz = L1 - L2*sin(th2) - L3*sin(th2 - th3);
    px = off1*cos(th1) - off2*sin(th1) + L2*cos(th1)*cos(th2) + L3*cos(th1)*cos(th2)*cos(th3) + L3*cos(th1)*sin(th2)*sin(th3);
    py = off2*cos(th1) + off1*sin(th1) + L2*cos(th2)*sin(th1) + L3*cos(th2)*cos(th3)*sin(th1) + L3*sin(th1)*sin(th2)*sin(th3);
}

ThreeLinks::~ThreeLinks(){/*nothing to destruct*/}

void ThreeLinks::Jacobian()
{
  
    J[0][0] = -sin(th1)/(L2*cos(th2) + L3*cos(th2 - th3));
    J[0][1] = cos(th1)/(L2*cos(th2) + L3*cos(th2 - th3));
    J[0][2] = 0;
    J[1][0] = -(cos(th2 - th3)*cos(th1))/(L2*sin(th3));
    J[1][1] = -(cos(th2 - th3)*sin(th1))/(L2*sin(th3));
    J[1][2] = sin(th2 - th3)/(L2*sin(th3));             // changed
    J[2][0] = -(cos(th1)*(L2*cos(th2) + L3*cos(th2 - th3)))/(L2*L3*sin(th3));
    J[2][1] = -(sin(th1)*(L2*cos(th2) + L3*cos(th2 - th3)))/(L2*L3*sin(th3));
    J[2][2] = (L2*sin(th2) + L3*sin(th2 - th3))/(L2*L3*sin(th3));     //changed
  
  /*
    J[0][0] = -sin(th1)/(off1 + L2*cos(th2) + L3*cos(th2 - th3));
    J[0][1] = cos(th1)/(off1 + L2*cos(th2) + L3*cos(th2 - th3));
    J[0][2] = 0; 
    J[1][0] = -(2*(L3*cos(th1) - L3*cos(th1)*cos(th2)^2 - L3*cos(th1)*cos(th3)^2 + off1*cos(th1)*cos(th2)*cos(th3) - off2*cos(th2)*cos(th3)*sin(th1) + off1*cos(th1)*sin(th2)*sin(th3) - off2*sin(th1)*sin(th2)*sin(th3) + L2*cos(th1)*cos(th2)^2*cos(th3) + 2*L3*cos(th1)*cos(th2)^2*cos(th3)^2 + L2*cos(th1)*cos(th2)*sin(th2)*sin(th3) + 2*L3*cos(th1)*cos(th2)*cos(th3)*sin(th2)*sin(th3)))/(2*L2*off1*sin(th3) + 2*L2^2*cos(th2)*sin(th3) + L2*L3*sin(th2) - L2*L3*cos(2*th3)*sin(th2) + L2*L3*sin(2*th3)*cos(th2));
    J[1][1] = -(2*(L3*sin(th1) - L3*cos(th2)^2*sin(th1) - L3*cos(th3)^2*sin(th1) + 2*L3*cos(th2)^2*cos(th3)^2*sin(th1) + off2*cos(th1)*cos(th2)*cos(th3) + off1*cos(th2)*cos(th3)*sin(th1) + off2*cos(th1)*sin(th2)*sin(th3) + off1*sin(th1)*sin(th2)*sin(th3) + L2*cos(th2)^2*cos(th3)*sin(th1) + L2*cos(th2)*sin(th1)*sin(th2)*sin(th3) + 2*L3*cos(th2)*cos(th3)*sin(th1)*sin(th2)*sin(th3)))/(2*L2*off1*sin(th3) + 2*L2^2*cos(th2)*sin(th3) + L2*L3*sin(th2) - L2*L3*cos(2*th3)*sin(th2) + L2*L3*sin(2*th3)*cos(th2));
    J[1][2] = sin(th2 - th3)/(L2*sin(th3));
    J[2][0] = -(2*(L3^2*cos(th1) + L2^2*cos(th1)*cos(th2)^2 - L3^2*cos(th1)*cos(th2)^2 - L3^2*cos(th1)*cos(th3)^2 + 2*L3^2*cos(th1)*cos(th2)^2*cos(th3)^2 + L2*off1*cos(th1)*cos(th2) - L2*off2*cos(th2)*sin(th1) + L3*off1*cos(th1)*cos(th2)*cos(th3) - L3*off2*cos(th2)*cos(th3)*sin(th1) + L3*off1*cos(th1)*sin(th2)*sin(th3) - L3*off2*sin(th1)*sin(th2)*sin(th3) + 2*L2*L3*cos(th1)*cos(th2)^2*cos(th3) + 2*L3^2*cos(th1)*cos(th2)*cos(th3)*sin(th2)*sin(th3) + 2*L2*L3*cos(th1)*cos(th2)*sin(th2)*sin(th3)))/(L2*L3*(L3*sin(th2) + 2*off1*sin(th3) + 2*L2*cos(th2)*sin(th3) - L3*cos(2*th3)*sin(th2) + L3*sin(2*th3)*cos(th2)));
    J[2][1] = -(2*(L3^2*sin(th1) + L2^2*cos(th2)^2*sin(th1) - L3^2*cos(th2)^2*sin(th1) - L3^2*cos(th3)^2*sin(th1) + 2*L3^2*cos(th2)^2*cos(th3)^2*sin(th1) + L2*off2*cos(th1)*cos(th2) + L2*off1*cos(th2)*sin(th1) + L3*off2*cos(th1)*cos(th2)*cos(th3) + L3*off1*cos(th2)*cos(th3)*sin(th1) + L3*off2*cos(th1)*sin(th2)*sin(th3) + L3*off1*sin(th1)*sin(th2)*sin(th3) + 2*L2*L3*cos(th2)^2*cos(th3)*sin(th1) + 2*L3^2*cos(th2)*cos(th3)*sin(th1)*sin(th2)*sin(th3) + 2*L2*L3*cos(th2)*sin(th1)*sin(th2)*sin(th3)))/(L2*L3*(L3*sin(th2) + 2*off1*sin(th3) + 2*L2*cos(th2)*sin(th3) - L3*cos(2*th3)*sin(th2) + L3*sin(2*th3)*cos(th2)));
    J[2][2] = (L2*sin(th2) + L3*sin(th2 - th3))/(L2*L3*sin(th3));
   */
}

void ThreeLinks::update(Path *path_des, int i)
{
    float err_x;
    float err_y;
    float err_z;
    err_x = path_des->x[i] - px;
    err_y = path_des->y[i] - py;
    err_z = path_des->z - pz;
    th1 = th1 + k_cl*(J[0][0]*err_x + J[0][1]*err_y + J[0][2]*err_z);  // without v
    th2 = th2 + k_cl*(J[1][0]*err_x + J[1][1]*err_y + J[1][2]*err_z);
    th3 = th3 + k_cl*(J[2][0]*err_x + J[2][1]*err_y + J[2][2]*err_z);
    //px = cos(th1)*(L2*cos(th2) + L3*cos(th2 - th3));
    //py = sin(th1)*(L2*cos(th2) + L3*cos(th2 - th3));
    pz = L1 - L2*sin(th2) - L3*sin(th2 - th3);
    px = off1*cos(th1) - off2*sin(th1) + L2*cos(th1)*cos(th2) + L3*cos(th1)*cos(th2)*cos(th3) + L3*cos(th1)*sin(th2)*sin(th3);
    py = off2*cos(th1) + off1*sin(th1) + L2*cos(th2)*sin(th1) + L3*cos(th2)*cos(th3)*sin(th1) + L3*sin(th1)*sin(th2)*sin(th3);
    Jacobian();
}
/********************************************************/
/*
sturct Point
{
	float x[N];
	float y[N];
	float z[N];
};
sturct Joint
{
	float th1;
	float th2;
	float th3;
}q;
sturct Point p_out;
sturct Point circle;

// caLcuLate the coordinate of end-effector
void forward_kin(sturct Joint *q, sturct Point *point, int i)
{
	point.x[i] = cos(q.th1)*(L2*cos(q.th2) + L3*cos(q.th2 - q.th3));
	point.y[i] = sin(q.th1)*(L2*cos(q.th2) + L3*cos(q.th2 - q.th3));
	point.z[i] = L1 + L2*sin(q.th2) + L3*sin(q.th2 - q.th3);
}
*/
// update Jocobian
/*
void Jacobian(sturct Joint *q)
{
	J[0][0] = -sin(q.th1)/(L2*cos(q.th2) + L3*cos(q.th2 - q.th3));
	J[0][1] = cos(q.th1)/(L2*cos(q.th2) + L3*cos(q.th2 - q.th3));
	J[0][2] = 0;
	J[1][0] = -(cos(q.th2 - q.th3)*cos(q.th1))/(L2*sin(q.th3));
	J[1][1] = -(cos(q.th2 - q.th3)*sin(q.th1))/(L2*sin(q.th3));
	J[1][2] = -sin(q.th2 - q.th3)/(L2*sin(q.th3));
	J[2][0] = -(cos(q.th1)*(L2*cos(q.th2) + L3*cos(q.th2 - q.th3)))/(L2*L3*sin(q.th3));
	J[2][1] = -(sin(q.th1)*(L2*cos(q.th2) + L3*cos(q.th2 - q.th3)))/(L2*L3*sin(q.th3));
	J[2][2] = -(L2*sin(q.th2) + L3*sin(q.th2 - q.th3))/(L2*L3*sin(q.th3));
}
*/

// feedback controL
/*
void update_q(sturct Joint *q, sturct Point *p_des, sturct Point *p_out, int i)
{
	fLoat err_x;
	fLoat err_y;
	fLoat err_z;
	err_x = p_des.x[i] - p_out.x[i];
	err_y = p_des.y[i] - p_out.y[i];
	err_z = p_des.z[i] - p_out.z[i];
	q.th1 = q.th1 + k_cl*(J[0][0]*err_x + J[0][1]*err_y + J[0][2]*err_z);  // without v
	q.th2 = q.th2 + k_cl*(J[1][0]*err_x + J[1][1]*err_y + J[1][2]*err_z);
	q.th3 = q.th3 + k_cl*(J[2][0]*err_x + J[2][1]*err_y + J[2][2]*err_z);
}
*/
