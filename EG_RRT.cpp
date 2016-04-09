#include <iostream>
#include <vector>
#include <cmath>
float t=2;

std::vector<double> Node::Integrate(Node *  x_v,std::vector<float> a)

{
std::vector<double> pose(3);
for(int j=0;j<=2;i++)
{
 pose[j]=(x_v->linear_v[j]*t + 0.5 a[j]*t*t);
}
  return pose;
}

void Node::Update_CVF(Node * x_v)
{
int cnt =1;
x_v->CVF=1/(1000);//fix the size
while(x_v!=NULL)
{ cnt=cnt+1;
  x_v=x_v->parent;
  x_v->CVF=1/(pow(1000,cnt));
}

}

Node* BestInput(Node * &x_v,int & u )
{
std::vector<double> pose(3);// to change depending on the c space
//float x_v={0,0}
Node * x_curr=NULL;
pose=Integrate(x_v,u);

if (IsValid(pose)==true)
{
x_curr=new Node;
x_curr->pose=pose;
return x_curr;
}
else
{
x_v->cntrl_collision.push_back(u);
float d_min=99999999;
float d=10000000;
std::vector<float> u_best(3,0);

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
