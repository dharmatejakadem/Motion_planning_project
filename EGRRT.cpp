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
}
