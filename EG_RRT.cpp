#include <iostream>
#include <vector>
#include <cmath>
float t=2;

cntrl_set=



float ReachSetDist(std::vector<double> pose,Node * x ,int &u)
{
float dist=1000000;
float dist_new;
std::vector<double> pose_new;
for(int i =0;i<10000;i++)//size of control set not fixed
{
pose_new=Integrate(x,cntrol_set[i]);
dist_new=Dist(pose_new,pose);
if (dist_new < dist)
{
 dist=dist_new;
 u=cntrol_set[i];
}

}
return dist;
}

float Dist(std::vector<double> pose1,std::vector<double> pose2)
{
float result;
result=sqrt(pow((pose2[0]-pose1[0]),2)+pow((pose2[1]-pose1[1]),2)+pow((pose2[2]-pose1[2]),2));
return result;
}

std::vector<double> Node::Integrate(Node *  x_v,std::vector<float> a)
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
x_v->CVF=1/(1000);//fix the size
while(x_v!=NULL)
{ cnt=cnt+1;
  x_v=x_v->parent;
  x_v->CVF=1/(pow(1000,cnt));
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
x_curr->pose=pose;
x_curr->parent=x_v;
return x_curr;
}
else
{
x_v->cntrl_collision.push_back(u);
float d_min=99999999;
float d=10000000;
std::vector<float> u_best(3,NULL);

for(int i=0;i<cntrl.size();i++)
    {
    if( complete_cntrlset[i]!=u and (x_v->cntrl_collision.size()!=1000))//fix size of control set and how it is depictied
        {
        pose=Integrate(x_v->pose,cntrl_u[i]);
        d=Dist(pose,x_rand->pose);//x_rand not defined
        if (d<d_min)
        {
            if (IsValid(pose)==true)
            {   x_curr=new Node();
                d_min=d;
                x_curr->pre_controlset=complete_cntrlset[i];
                x_curr->pose=pose;
                x_curr->parent=x_v;
            }
            else
            { x_v->cntrl_collision.push_back(cntrl[i])
              Update_CVF(x_v);

            }

        }
        }
    else if (x_v->cntrl_collision.size()==1000)
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
if(t1[i]->cntrl_collision.size()<1000) and (t1[i]->CVF<Random())//write the random func //size of control set
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


