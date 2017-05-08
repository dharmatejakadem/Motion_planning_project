#include <iostream>
#include <vector>
//#include "EG_RRT.h"
#include <cmath>
float t=2;
std::vector <float> a;
a.push_back(-0.1971);
a.push_back(-0.185);
a.push_back(0.088);
std::vector<float> b;
b.push_back(-0.2054);
b.push_back(-0.2049);
b.push_back(-0.15);
std::vector<float> c;
c.push_back(-0.2145);
c.push_back(-0.0817);
c.push_back(-0.3888);
std::vector<float> d;
d.push_back(-0.2328);
d.push_back(-0.1370);
d.push_back(-0.3856);
std::vector<float> e;
e.push_back(-0.2248);
e.push_back(-0.1107);
e.push_back(0.5047);
std::vector<float> f;
f.push_back(-0.3028);
f.push_back(-0.1435);
f.push_back(-1.0368);
std::vector<std::vector<float> > control_set;
control_set.push_back(a);
control_set.push_back(b);
contrl_set.push_back(c);
control_set.push_back(d);
control_set.push_back(e);
control_set.push_back(f);


float tree::ReachSetDist(std::vector<double> pose,Node * x ,int &u)
{
float dist=1000000;
float dist_new;
std::vector<double> pose_new;
for(int i =0;i<6;i++)//size of control set not fixed
{
pose_new=Integrate(x,control_set[i]);
dist_new=Dist(pose_new,pose);
if (dist_new < dist)
{
 dist=dist_new;
 u=i;
}

}
return dist;
}

float tree::Dist(std::vector<double> pose1,std::vector<double> pose2)
{
float result;
result=sqrt(pow((pose2[0]-pose1[0]),2)+pow((pose2[1]-pose1[1]),2)+pow((pose2[2]-pose1[2]),2));
return result;
}

std::vector<double> tree::Integrate(Node *  x_v,std::vector<float> a)
{
std::vector<double> pose(3);
for(int j=0;j<=2;i++)
{
 pose[j]=(x_v->linear_v[j]*t + 0.5 a[j]*t*t);
}
  return pose;
}

void tree::Update_CVF(Node * x_v)
{
int cnt =1;
x_v->CVF=1/(6);//fix the size
while(x_v!=NULL)
{ cnt=cnt+1;
  x_v=x_v->parent;
  x_v->CVF=1/(pow(6,cnt));
}

}

Node* tree::BestInput(Node * &x_v,int & u )
{
std::vector<double> pose(3);// to change depending on the c space
//float x_v={0,0}
Node * x_curr=NULL;
pose=Integrate(x_v,u);

if (IsValid(pose)==true)
{
x_curr=new Node;
x_curr->position=pose;
x_curr->parent=x_v;
return x_curr;
}
else
{
x_v->cntrl_collision.push_back(u);
float d_min=99999999;
float d=10000000;
std::vector<float> u_best(3,NULL);

for(int i=0;i<control_set.size();i++)
    {
    if( control_set[i]!=control_set[u] and (x_v->cntrl_collision.size()!=6))//fix size of control set and how it is depictied
        {
        pose=Integrate(x_v->position,control_set[i]);
        d=Dist(pose,x_rand->pose);//x_rand not defined
        if (d<d_min)
        {
            if (IsValid(pose)==true)
            {   x_curr=new Node();
                d_min=d;
                x_curr->pre_controlset=i;
                x_curr->position=pose;
                x_curr->parent=x_v;
            }
            else
            { x_v->cntrl_collision.push_back(i);
              Update_CVF(x_v);

            }

        }
        }
    else if (x_v->cntrl_collision.size()==6)
    {
    return x_curr;
    }

}
return x_curr;
}
}

void tree::BestState(std::vector<double> pose,Node* &location,int &u_closest)  //here the location and u_closest
{location= NULL;
float drmin = 100000,dnmin = 100000,dn,dr;
int u;
for(int i =0; i<t1.size();i++)//t1 is tree object
{
if((t1[i]->cntrl_collision.size()<6) and (t1[i]->CVF<Random()))//write the random func //size of control set
{
dr=ReachSetDist(pose,t1[i],u);
dn=Dist(pose,t1[i]->position);
if (dr < drmin)
{
drmin=dr;
location=t1[i];
u_closest=u;
}
if (dn<dnmin)
{
dnmin=dn;
}

}


}
if (dnmin<drmin)
{ location=NULL;
 u_closest=NULL;

}

}
vector<RRT_Node <T>* > EGRRTbuild(T initial, T Goal,EnvironmentBasePtr a)
{
RRT_Node<T> *x;
T qrand;
int i=0;
x=new RRT_Node<T> (initial);
Tree<T> t1;
t1.addnode(x);
do
{
i++;
qrand=Random();
BestState(qrand,xbest,indexofu_closest);
if(xbest!=NULL)
{
BestInput(xbest,qrand,indexofu_closest);
if(indexofu_closest!=NULL)
{
t1.InsertNode(xbest) //InsertNode is a function
}
}

}while(i<100000 && Euclidean(t1.getnodes()[t1.getnodes().size()-1]->getval(), Goal) >= 0.25);
t1.addnode(new RRT_Node<T> (Goal,t1.getnodes()[t1.getnodes().size()-1]));
return t1.getpath(t1.getnodes()[t1.getnodes().size()-1]);
}

