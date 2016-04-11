
#ifndef RRT_H
#define RRT_H

#include <iostream>
#include <stdio.h>
#include <vector>
//#include <openrave/plugin.h>
#include <boost/bind.hpp>
using namespace OpenRAVE;
using namespace std;
typedef double dReal;



class RRTNode
{
std::vector<double> _configuration;
RRTNode* parent;

public:
std::vector <double> position;
int pre_controlset;//control set it used to reach this state
float CVF;
std::vector<double> linear_v; //3 linear velocity
std::vector<int> cntrl_collision; //control values that caused collision
RRTNode*  getParent();
RRTNode();
~RRTNode();
void setParent(RRTNode* p);
RRTNode(std::vector<double> ele);
RRTNode(std::vector<double> ele,RRTNode* p);
std::vector<double> getConfig();

};


class NodeTree
{
std::vector<RRTNode*> _nodes;
public:
std::vector<RRTNode*> n;
RRTNode* BestInput(RRTNode * &x_v,int &u );
void Update_CVF(RRTNode * x_v);
std::vector<double> Integrate(RRTNode*,std::vector<float>);
void BestState(std::vector<double> pose,RRTNode* &location,int &u_closest);
float ReachSetDist(std::vector<double> pose,RRTNode* x ,int &u);
float Dist(std::vector<double> pose1,std::vector<double> pose2);
std::vector<double> Integrate(RRTNode,std::vector<float> );
void add(RRTNode &ele);
void add(RRTNode* ele);
void delNode(int ele);
void delNode(RRTNode &ele);
void delBranch(RRTNode *start,RRTNode *endNode);
std::vector< std::vector<double> >  getNodes();
std::vector<RRTNode*> getTree();
std::vector<RRTNode*> getPathToStart(RRTNode* ele);
void display();
NodeTree();
NodeTree(RRTNode &ele);
};



bool inRegion(vector<double> inp);
void display(vector<float> inp);
void display(vector<dReal> inp);
vector<double> randomSample();
vector<double> subtractMul(vector<double> a,vector<double> &b, float scale);
vector<double> add(vector<double> a,vector<double> &b);
float norm(vector<double> a);
float euclidean(vector<double> a,vector<double> b,bool flag );
RRTNode* nearestNeighbor(std::vector<RRTNode*> tree, vector<double> inputConfig);
std::vector< RRTNode* > RRTplanner(OpenRAVE::EnvironmentBasePtr env, vector<double> goalConfig,float step,float goalBias);
float normalize_angle(float q);
float metric_weight[] = {1,1,1,1,1,1,1};
std::vector<double> lower;
std::vector<double> upper;
#include "kdtree.cpp"


#endif
