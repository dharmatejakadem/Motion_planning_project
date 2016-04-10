#include <iostream>
#include <vector>
#include <cmath>




class Node
{public:
std::vector <double> position;
int pre_controlset;//control set it used to reach this state
float CVF;
std::vector<double> linear_v; //3 linear velocity
std::vector<int> cntrl_collision; //control values that caused collision
Node* parent;
};


class tree
{public:
std::vector<Node*> n;
Node* BestInput(Node * &x_v,int &u );
void Update_CVF(Node * x_v);
void BestState(std::vector<double> pose,Node* &location,int &u_closest);
float ReachSetDist(std::vector<double> pose,Node * x ,int &u);
float Dist(std::vector<double> pose1,std::vector<double> pose2);
std::vector<double> Integrate(Node* & , std::vector<float> );
};
