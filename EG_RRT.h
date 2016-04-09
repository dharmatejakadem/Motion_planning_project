#include <iostream>
#include <vector>
#include <cmath>

struct Node
{
std::vector<double> position(7);
int pre_controlset;//control set it used to reach this state
float CVF;
std::vector<double> linear_v; //3 linear velocity
std::vector<int> cntrl_collision; //control values that caused collision
Node* parent;
};


struct tree
{
std::vector<Node*> n;
Node* BestInput(Node * &x_v,int &u );
void Node::Update_CVF(Node * x_v);
std::vector<double> Node::Integrate(Node *  x_v,std::vector<float> a);


};
