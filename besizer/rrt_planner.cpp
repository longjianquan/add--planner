#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>

#include <time.h>
#include <omp.h>

#include <pluginlib/class_list_macros.h>

#include "inspire_rrt_planner.h"

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt_plan::rrt_planner, nav_core::BaseGlobalPlanner)

static const float INFINIT_COST = INT_MAX; //!< cost of non connected nodes

int mapSize;
bool* OGM;
bool* OGM2;
RRT myRRT1;//RRT树结构
RRT myRRT2;
ofstream irrtlog("/home/ljq/irrtlog.txt");

int clock_gettime(clockid_t clk_id, struct timespect *tp);

long getCurrentTime()
{
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return tv.tv_sec*1000+tv.tv_usec/1000;
}

timespec diff(timespec start, timespec end)
{
    timespec temp;
    if ((end.tv_nsec-start.tv_nsec)<0)
    {
        temp.tv_sec = end.tv_sec-start.tv_sec-1;
        temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
    }
    else
    {
        temp.tv_sec = end.tv_sec-start.tv_sec;
        temp.tv_nsec = end.tv_nsec-start.tv_nsec;
    }
    return temp;
}

namespace rrt_plan
{
//Default Constructor
rrt_planner::rrt_planner():initialized_(false)
{

}
rrt_planner::rrt_planner(ros::NodeHandle &nh)
{
    ROSNodeHandle = nh;
}

rrt_planner::rrt_planner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    initialize(name, costmap_ros);
}

void rrt_planner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{

    if (!initialized_)
    {
        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros_->getCostmap();

        ros::NodeHandle private_nh("~/" + name);

        originX = costmap_->getOriginX();
        originY = costmap_->getOriginY();

        width = costmap_->getSizeInCellsX();
        height = costmap_->getSizeInCellsY();
        resolution = costmap_->getResolution();
        mapSize = width*height;

        ROS_INFO("---originX=%f,originY=%f---",originX,originY);

        ROS_INFO("---width=%d,height=%d---",width,height);

        ROS_INFO("---resolution=%f---",resolution);

        plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
        plan_pub_2 = private_nh.advertise<nav_msgs::Path>("plan2", 1);

        OGM = new bool [mapSize];
        for (unsigned int iy = 0; iy < costmap_->getSizeInCellsY(); iy++)
        {
            for (unsigned int ix = 0; ix < costmap_->getSizeInCellsX(); ix++)
            {
                unsigned int cost = static_cast<int>(costmap_->getCost(ix, iy));

                //if (cost == 0)
                /*if(cost<=80)  // 调节避障条件，范围越大，要求越松散
                    OGM[iy*width+ix]=true;
                else
                    OGM[iy*width+ix]=false;*/
              if(cost<=78)  
                    OGM[iy*width+ix]=true;
                else
                    OGM[iy*width+ix]=false;
            }
        }

        OGM2 = new bool [mapSize];
        for (unsigned int iy = 0; iy < costmap_->getSizeInCellsY(); iy++)
        {
            for (unsigned int ix = 0; ix < costmap_->getSizeInCellsX(); ix++)
            {
                unsigned int cost = static_cast<int>(costmap_->getCost(ix, iy));

                if(cost==254)  // 调节避障条件，范围越大，要求越松散
                    OGM2[iy*width+ix]=false;
                else
                    OGM2[iy*width+ix]=true;
            }
        }
        
        ROS_INFO("---rrt planner1 initialized successfully---");

        initialized_ = true;
    }
    else
    {
        ROS_WARN("---This planner1 has already been initialized... doing nothing---");
    }
}



/*void rrt_planner::getGridVal(vector<vector<float> >& gridVal, int startRowID, int startColID,int goalRowID,int goalColID)  //模拟温度场
{
    int subwidth=width/mapReduceSize,subheight=height/mapReduceSize;
    int i , j , m , n , s;
    float gridMap[subheight][subwidth];

    #pragma opm parallel for private(i,j) schedule(static)
    for (i=0; i<subheight; i++)
    {
        for (j=0; j<subwidth; j++)
        {
            gridMap[i][j] = 1.0f;  //1表示自由区域

            gridVal[i][j] = 0.0f;
            for (m=0; m<mapReduceSize; m++)
            {
                for (n=0; n<mapReduceSize; n++)
                {
                    int cellIndex=getCellIndex(i*mapReduceSize+m,j*mapReduceSize+n);
                    if(!OGM2[cellIndex])
                    {
                        gridMap[i][j] = 0.0f;  //0表示障碍物
                    }
                }
            }
        }
    }
    

    int initIdx[2] = {0}; // index of initial point
    int goalIdx[2] = {0}; // index of goal point

    getIdx(startRowID, startColID);
    getIdx(goalRowID, goalColID);

    initIdx[0]=startRowID;
    initIdx[1]=startColID;
    goalIdx[0]=goalRowID;
    goalIdx[1]=goalColID;

    gridVal[initIdx[0]][initIdx[1]] = initialTemperature;
    gridVal[goalIdx[0]][goalIdx[1]] = -initialTemperature;
    
    
    #pragma omp parallel for private(s,i,j) schedule(static)
    for (s=0; s<iterationTime; s++)
    {
        {
            for (i=1; i<subheight-1; i++)
            {
                for (j=1; j<subwidth-1; j++)
                {
                    if (gridMap[i][j])
                    {
                        float delta = (gridVal[i-1][j]*gridMap[i-1][j]
                                       +gridVal[i][j-1]*gridMap[i][j-1]
                                       +gridVal[i+1][j]*gridMap[i+1][j]
                                       +gridVal[i][j+1]*gridMap[i][j+1])
                                      /(gridMap[i-1][j]+gridMap[i][j-1]+gridMap[i+1][j]+gridMap[i][j+1]);
                        gridVal[i][j] = delta;
                    }
                }
            }
        }
    }
}*/



bool rrt_planner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                           std::vector<geometry_msgs::PoseStamped>& plan)
{
    srand((unsigned)time(NULL));//设置随机数种子

    if (!initialized_)
    {
        ROS_ERROR("---The planner has not been initialized, please call initialize() to use the planner---");
        return false;
    }

    if (goal.header.frame_id != costmap_ros_->getGlobalFrameID())
    {
        ROS_ERROR("---This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.---",
                  costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
        return false;
    }


    plan.clear();

    tf::Stamped < tf::Pose > goal_tf;
    tf::Stamped < tf::Pose > start_tf;

    poseStampedMsgToTF(goal, goal_tf);
    poseStampedMsgToTF(start, start_tf);

    // convert the start and goal positions

    float startX = start.pose.position.x;
    float startY = start.pose.position.y;

    float goalX = goal.pose.position.x;
    float goalY = goal.pose.position.y;

    getCorrdinate(startX, startY);
    getCorrdinate(goalX, goalY);

    int startCell;
    int goalCell;

    //ROS_INFO("---startX=%f,startY=%f---",startX,startY);
    //ROS_INFO("---goalX=%f,goalY=%f---",goalX,goalY);

    if (isCellInsideMap(startX, startY) && isCellInsideMap(goalX, goalY))
    {
        startCell = convertToCellIndex(startX, startY);
        goalCell = convertToCellIndex(goalX, goalY);
        //cout << startCell <<"\t"<< start.pose.position.x <<"\t"<< start.pose.position.y <<"\t"
            //<< goalCell <<"\t"<< goal.pose.position.x <<"\t"<< goal.pose.position.y<<"\n";
    }
    else
    {
        ROS_WARN("---the start or goal is out of the map---");
        //rrtlog<<"---the start or goal is out of the map---"<<endl;
        return false;
    }

    //cout<<"startposion:"<<getCellRowID(startCell)<<"  "<<getCellColID(startCell)<<endl;
    //cout<<"goalposion:"<<getCellRowID(goalCell)<<"  "<<getCellColID(goalCell)<<endl;

    vector<RRT::rrtNode> rrtPath;

    if (isStartAndGoalCellsValid(startCell, goalCell))
    {
     
	       long t1,t2,t3;
	        t1=getCurrentTime();
            cout<<t1<<endl;
            //timespec time1, time2, time3;
            /* take current time here */
            //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);

           /* vector<vector<float> > gridVal;
            gridVal.clear();

            gridVal.resize(height/mapReduceSize);
            for(int i=0; i<(height/mapReduceSize); i++)
                gridVal[i].resize(width/mapReduceSize);

            getGridVal(gridVal,getCellRowID(startCell), getCellColID(startCell),getCellRowID(goalCell), getCellColID(goalCell));
            //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
             t2=getCurrentTime();
            
            
            irrtlog<<"+++++++++++++++++++++++++++++++"<<endl;
            for(int k=0; k<(height/mapReduceSize); k++)
            {
                for(int l=0; l<(width/mapReduceSize-1); l++)
                {
                    irrtlog<<gridVal[k][l]<<" ";
                }
                irrtlog<<gridVal[k][width/mapReduceSize]<<endl;
                //irrtlog<<endl;
            }
            irrtlog<<"-------------------------------"<<endl;*/
            
            

            myRRT1.clearRRT();
            myRRT2.clearRRT();
            RRT temprrt1(getCellRowID(startCell),getCellColID(startCell));
            RRT temprrt2(getCellRowID(goalCell),getCellColID(goalCell));
            myRRT1=temprrt1;
            myRRT2=temprrt2;

            //rrtPath=findCellPath(myRRT, startCell, goalCell, gridVal);
            rrtPath=findCellPath(myRRT1,myRRT2, startCell, goalCell);
            //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time3);
            t2=getCurrentTime();
            cout<<t1<<endl;
            
            //cout<<"temperature field time="<<t2-t1<<endl;
            //irrtlog<<"temperature field time="<<t2-t1<<endl;
            //cout<<"planning time="<<t3-t2<<endl;
            //irrtlog<<"planning time="<<t3-t2<<endl;
            cout<<"total time="<<t2-t1<<endl;
    }
    else
    {
        ROS_WARN("------Not valid start or goal------");
        return false;
    }

    //if the global planner find a path
    if (rrtPath.size()>0)
    {
        convertToPlan(rrtPath,plan,goal,goalCell);

        float path_length = 0.0;

        std::vector<geometry_msgs::PoseStamped>::iterator it = plan.begin();

        geometry_msgs::PoseStamped last_pose;
        last_pose = *it;
        it++;
        irrtlog<<"路径位姿点："<<endl;
        for (; it!=plan.end(); ++it)
        {
            irrtlog<<"("<<(*it).pose.position.x<<","<<(*it).pose.position.y<<")"<<endl;
            path_length += hypot(  (*it).pose.position.x - last_pose.pose.position.x,
                                   (*it).pose.position.y - last_pose.pose.position.y );
            last_pose = *it;
        }
        cout <<"------The global path length: "<< path_length<< " meters------"<<endl;
        irrtlog <<"------The global path length: "<< path_length<< " meters------"<<endl;
    }
    else
    {
        ROS_WARN("------The planner failed to find a path, choose other goal position------");
        //return false;
    }

    publishPlan(plan);
    

    //cout<<"+++++++++++++++++++++++++++++++"<<endl;
    irrtlog<<"+++++++++++++++++++++++++++++++"<<endl;
    //cout<<"树中所有结点(格式：结点ID，该结点的父结点ID，x坐标值，y坐标值):"<<endl;
    irrtlog<<"树中所有结点(格式：结点ID，该结点的父结点ID，x坐标值，y坐标值):"<<endl;
    RRT::rrtNode  l;
    for(int m=0; m<myRRT1.getTreeSize(); m++)
    {
        l=myRRT1.getNode(m);
        //cout<<l.nodeID<<","<<l.parentID<<","<<l.posX* resolution+originX<<","<<l.posY* resolution+originY<<endl;
        irrtlog<<l.nodeID<<","<<l.parentID<<","<<l.posX* resolution+originX<<","<<l.posY* resolution+originY<<endl;
    }
    //cout<<"+++++++++++++++++++++++++++++++"<<endl;
    irrtlog<<"+++++++++++++++++++++++++++++++"<<endl;

    return !plan.empty();
}


void  rrt_planner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path)
{
    if (!initialized_)
    {
        ROS_ERROR(
            "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    if (!path.empty())
    {
        gui_path.header.frame_id = path[0].header.frame_id;
        gui_path.header.stamp = path[0].header.stamp;
    }

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++)
    {
        gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
}



//vector<int>   rrt_planner::findCellPath(RRT &myRRT, int startCell, int goalCell,vector<vector<float> > gridVal)  //使用温度场作为启发因子
vector<RRT::rrtNode>   rrt_planner::findCellPath(RRT &myRRT1,RRT &myRRT2, int startCell, int goalCell) 
{
    vector<int>  rrtPath;
    vector<int>  rrtPath1;
    vector<int>  rrtPath2;
    vector<RRT::rrtNode> randNodes;
    vector<RRT::rrtNode> randNode1s;
    vector<RRT::rrtNode> trees;

    bool addNodeResult = false, nodeToGoal = false;
    int u=0;
    while(u<8000)
    {
    if(u%2==0)
    {
        randNodes.clear();
        //generateTempPoint(tempNode,width, height);  //一次产生一个随机结点
        generateRandNodes( myRRT1,randNodes,width, height,getCellRowID(goalCell), getCellColID(goalCell));  //一次产生一个随机结点数组

      
        addNodeResult = addNewNodetoRRT(myRRT1,randNodes,getCellRowID(goalCell), getCellColID(goalCell),rrtStepSize); 
       //cout<<"  1     "<<addNodeResult<<endl;
        if(addNodeResult)
        {
            //nodeToGoal = checkNodetoGoal(getCellRowID(goalCell), getCellColID(goalCell),tempNode);//经典RRT方法添加新结点
            
            RRT::rrtNode lastNode=myRRT1.getNode(myRRT1.getTreeSize()-1);
            
            //RRT::rrtNode lastNode1=myRRT2.getNode(myRRT2.getTreeSize()-1);           
            //nodeToGoal=checkConnect(lastNode1,lastNode);
            nodeToGoal=checkConnect(myRRT2,lastNode);
            
            //cout<<"nodeToGoal1:"<<nodeToGoal<<endl;
            if(nodeToGoal)            
            {
                int index;
                vector<int>v1;

               for(int i=0;i<myRRT2.getTreeSize();i++)
               {
                    double distance = sqrt(pow(lastNode.posX-myRRT2.getNode(i).posX,2)+pow(lastNode.posY-myRRT2.getNode(i).posY,2));
                if((distance < rrtStepSize))
                {
                    index=i;
                    v1.push_back(index);                
                }
               }
                RRT::rrtNode lastNode1=myRRT2.getNode(v1[0]);
                rrtPath1 = myRRT1.getRootToEndPath(lastNode.nodeID);//启发函数优化RRT方法得到路径
                rrtPath2 = myRRT2.getRootToEndPath(lastNode1.nodeID);

                break;
            }

        }
    }
    else
    {
        randNode1s.clear();
        //generateTempPoint(tempNode,width, height);  //一次产生一个随机结点
        generateRandNodes( myRRT2,randNode1s,width, height,getCellRowID(startCell), getCellColID(startCell));  //一次产生一个随机结点数组

        //addNodeResult = addNewPointtoRRT(myRRT,tempNode,rrtStepSize);  //经典RRT方法添加新结点
        //addNodeResult = addNewNodetoRRT(myRRT,randNodes,getCellRowID(goalCell), getCellColID(goalCell),rrtStepSize,gridVal); //启发函数优化RRT方法
        addNodeResult = addNewNodetoRRT(myRRT2,randNode1s,getCellRowID(goalCell), getCellColID(goalCell),rrtStepSize); 
        //cout<<"  2     "<<addNodeResult<<endl;
        if(addNodeResult)
        {
            //nodeToGoal = checkNodetoGoal(getCellRowID(goalCell), getCellColID(goalCell),tempNode);//经典RRT方法添加新结点

            RRT::rrtNode lastNode=myRRT2.getNode(myRRT2.getTreeSize()-1);         
            //RRT::rrtNode lastNode1=myRRT1.getNode(myRRT1.getTreeSize()-1);           
            //nodeToGoal=checkConnect(lastNode1,lastNode);
            //cout<<"nodeToGoal2:"<<nodeToGoal<<endl;
            nodeToGoal=checkConnect(myRRT1,lastNode);
                //path = myRRT.getRootToEndPath(tempNode.nodeID);//经典RRT方法得到路径
            if(nodeToGoal)          
            {
                int index;
                vector<int>v2;
               for(int i=0;i<myRRT1.getTreeSize();i++)
               {
                double distance = sqrt(pow(lastNode.posX-myRRT1.getNode(i).posX,2)+pow(lastNode.posY-myRRT1.getNode(i).posY,2));
                if((distance < rrtStepSize))
                {
                    
                    index=i;
                    v2.push_back(index);
                }
               }
                RRT::rrtNode lastNode1=myRRT1.getNode(v2[0]);
                rrtPath2 = myRRT2.getRootToEndPath(lastNode.nodeID);//启发函数优化RRT方法得到路径
                rrtPath1 = myRRT1.getRootToEndPath(lastNode1.nodeID);
                break;
            }
        }

    }
             u++;
    }
    for(int i=0;i<rrtPath1.size();i++)
    {
        cout<<"tree1: "<<myRRT1.getNode(rrtPath1[i]).posX<<" "<<myRRT1.getNode(rrtPath1[i]).posY<<endl;
        trees.push_back(myRRT1.getNode(rrtPath1[i]));
    }
    for(int j=rrtPath2.size()-1;j>-1;j--)
    {
        cout<<"tree2: "<<myRRT2.getNode(rrtPath2[j]).posX<<" "<<myRRT2.getNode(rrtPath2[j]).posY<<endl;
        trees.push_back(myRRT2.getNode(rrtPath2[j]));
         
    }

    cout<<"RRT nodeSum="<<myRRT1.getTreeSize()<<endl;
    cout<<"RRT nodeSum="<<myRRT2.getTreeSize()<<endl;
    //irrtlog<<"RRT nodeSum="<<myRRT.getTreeSize()<<endl;
   

    return trees;
}
vector<RRT::rrtNode>rrt_planner::bersier(vector<RRT::rrtNode>rrtPath)
{
    vector<RRT::rrtNode>finalpath;
    RRT::rrtNode temp;
    RRT::rrtNode startNode;
    RRT::rrtNode point1;
    RRT::rrtNode point2;
    RRT::rrtNode point11;
    RRT::rrtNode point22;

    startNode=rrtPath[0];
    finalpath.push_back(startNode);
    for(int i=0;i<rrtPath.size()-2;i++)
    {
        float dist1=sqrt(pow(rrtPath[i].posX-rrtPath[i+1].posX,2)+pow(rrtPath[i].posY-rrtPath[i+1].posY,2));
        float dist2=sqrt(pow(rrtPath[i+2].posX-rrtPath[i+1].posX,2)+pow(rrtPath[i+2].posY-rrtPath[i+1].posY,2));
        point11=rrtPath[i+1];
        point22=rrtPath[i+1];
        double theta1=atan2(rrtPath[i].posY-rrtPath[i+1].posY,rrtPath[i].posX-rrtPath[i+1].posX);
        double theta2=atan2(rrtPath[i+2].posY-rrtPath[i+1].posY,rrtPath[i+2].posX-rrtPath[i+1].posX);
        bool s=true;
        int ss=1;
        while(s)
        {
            point1.posX=rrtPath[i+1].posX+ss*cos(theta1);
            point1.posY=rrtPath[i+1].posY+ss*sin(theta1);
            point2.posX=rrtPath[i+1].posX+ss*cos(theta2);
            point2.posY=rrtPath[i+1].posY+ss*sin(theta2);
            float dist3=sqrt(pow(point1.posX-rrtPath[i+1].posX,2)+pow(point1.posY-rrtPath[i+1].posY,2));
            float dist4=sqrt(pow(point2.posX-rrtPath[i+1].posX,2)+pow(point2.posY-rrtPath[i+1].posY,2));
            if((dist3>(dist1/2))||(dist4>(dist2/2)))
            {
                break;
            }

            if(pointcheck(point1,point2))
            {
                point11=point1;
                point22=point2;
                ss=ss+1;
            }
            else
            {
                s=false;
            }

        }
        for(float t=0.00;t<=1.00;)
        {
            temp.posX=point11.posX*pow(1-t,2)+2*(1-t)*t*rrtPath[i+1].posX+point22.posX*pow(t,2);
            temp.posY=point11.posY*pow(1-t,2)+2*(1-t)*t*rrtPath[i+1].posY+point22.posY*pow(t,2);
            //cout<<" beriser"<<temp.posX<<" "<<temp.posY<<endl;;
            finalpath.push_back(temp);            
            t=t+0.2;
        }

    }
    return finalpath;
}

void rrt_planner::convertToPlan(vector<RRT::rrtNode>rrtPath,std::vector<geometry_msgs::PoseStamped>& plan, geometry_msgs::PoseStamped  goal,int goalCell)
{
    vector<RRT::rrtNode>finalpath;
    /*for (int i = 0; i < rrtPath.size(); i++)
    {

        float x = 0.0;
        float y = 0.0;

        int tempNodeId=rrtPath[i];
        int cellIndex=getCellIndex(myRRT.getPosX(tempNodeId),myRRT.getPosY(tempNodeId));*/
       finalpath= bersier(rrtPath);
       for(int i=0;i<finalpath.size();i++)
        {
            float x=0.0;
            float y=0.0;
           int cellIndex=getCellIndex(finalpath[i].posX,finalpath[i].posY);
        convertToCoordinate(cellIndex, x, y);

        //cout<<"--cell"<<index<<": x="<<x<<",y="<<y<<endl;
        //rrtlog<<"--cell"<<index<<": x="<<x<<",y="<<y<<endl;

        geometry_msgs::PoseStamped pose = goal;


        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;

        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;

        plan.push_back(pose);
    }


    //处理最后一个结点，即是目标点goal
    float final_x = 0.0;
    float final_y = 0.0;
    int final_index = goalCell;
    convertToCoordinate(final_index, final_x, final_y);
    //cout<<"--cell"<<final_index<<": final_x="<<final_x<<",final_y="<<final_y<<endl;
    //rrtlog<<"--cell"<<final_index<<": final_x="<<final_x<<",final_y="<<final_y<<endl;

    geometry_msgs::PoseStamped final_pose = goal;
    final_pose.pose.position.x = final_x;
    final_pose.pose.position.y = final_y;
    final_pose.pose.position.z = 0.0;
    final_pose.pose.orientation.x = 0.0;
    final_pose.pose.orientation.y = 0.0;
    final_pose.pose.orientation.z = 0.0;
    final_pose.pose.orientation.w = 1.0;
    plan.push_back(final_pose);  //最后一个结点，即是目标点

}

bool rrt_planner::checkIfOnObstacles(RRT::rrtNode &tempNode)
{
    if((tempNode.posX<width)&&(tempNode.posY<height))
    {
        if(OGM[getCellIndex(tempNode.posX,tempNode.posY)])
            return true;
        else
            return false;
    }
    else
        return false;
}
bool rrt_planner::pointcheck(RRT::rrtNode const &m,RRT::rrtNode const &n)
{
    RRT::rrtNode test;
    float p=0.00;
    float dist=sqrt(pow(m.posX-n.posX,2)+pow(m.posY-n.posY,2));
    double theta=atan2(n.posY-m.posY,n.posX-m.posX);
    while(p<=1)
    {
        test.posX=(int)(m.posX+p*dist*cos(theta));
        test.posY=(int)(m.posY+p*dist*sin(theta));
        if(!checkIfOnObstacles(test))
        {
            break;
            return false;
        }

        p=p+0.2;
    }
}


bool rrt_planner::isCellInsideMap(float x, float y)
{
    bool valid = true;

    if (x > (width * resolution) || y > (height * resolution))
        valid = false;

    return valid;
}

void rrt_planner::mapToWorld(double mx, double my, double& wx, double& wy)
{
    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    wx = costmap->getOriginX() + mx * resolution;
    wy = costmap->getOriginY() + my * resolution;
}

//verify if the cell(i,j) is free
bool  rrt_planner::isFree(int i, int j)
{
    int CellID = getCellIndex(i, j);
    return OGM[CellID];

}

//verify if the cell(i,j) is free
bool  rrt_planner::isFree(int CellID)
{
    return OGM[CellID];
}

bool rrt_planner::isStartAndGoalCellsValid(int startCell,int goalCell)
{
    bool isvalid=true;
    bool isFreeStartCell=isFree(startCell);
    bool isFreeGoalCell=isFree(goalCell);
    if (startCell==goalCell)
    {
        cout << "------The Start and the Goal cells are the same...------" << endl;
        isvalid = false;
    }
    else
    {
        isvalid=(isFreeStartCell&&isFreeGoalCell);
    }
    return isvalid;
}

}
