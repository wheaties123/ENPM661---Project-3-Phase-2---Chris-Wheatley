% Chris Wheatley
% ENPM661 Spring 2020
% Project #3 Phase #2

% This script will use the A* algorithm to simulate a rigid robot exploring 
%   an action space, starting from a user-specified
%   location all the way until the goal node is explored.  
%   Then the optimal path will be drawn.  

close all; 
clear all;

% Solicit input node configuration from user
fprintf('\n');
prompt = 'Enter the x,y STARTING node location and starting orientation in degrees (e.g. [1,1,30]): ';
start_node = input(prompt);
fprintf('\n');

% Solicit goal node configuration from user
fprintf('\n');
prompt1 = 'Enter the x,y GOAL node location (e.g. [181,94]): ';
goal_node = input(prompt1);
fprintf('\n');

% Solicit robot radius from user
fprintf('\n');
prompt2 = 'Enter robot radius (unit length): ';
r = input(prompt2);
fprintf('\n');

% Solicit obstacle clearance from user
fprintf('\n');
prompt3 = 'Enter obstacle clearance (unit length): ';
c = input(prompt3);
fprintf('\n');

% Solicit movement distance from user
fprintf('\n');
prompt4 = 'Enter step size of movements (unit distance) between 1 and 10: ';
dist = input(prompt4);
fprintf('\n');

% Solicit action theta from user
fprintf('\n');
prompt5 = 'Enter angle between consecutive action (in degrees): ';
theta = input(prompt5);
fprintf('\n');

xmax=300; ymax=200;
fig=figure; hold on; axis equal;
% Define black border around action movement area
line([0 xmax xmax 0 0],[0 0 ymax ymax 0],'Color','black');
xlabel('X Coordinates'); ylabel('Y Coordinates');
title('A* Rigid (Pink Circle = Start ; Pink Triangle = Goal)');

xl = [0 xmax]; 
yl = [0 ymax];

% Define obstacle space
% Concave polygon
h1=line([20 50 75 100 75 25 20],[120 150 120 150 185 185 120],'Color','blue');

% Rectangle
h2=line([95 100 35 30 95],[30 38.6603 76.1603 67.5 30],'Color','blue');

% Diamond
h3=line([225 250 225 200 225],[10 25 40 25 10],'Color','blue');

% Ellipse
a=40; % horizontal radius
b=20; % vertical radius
x0=150; % x0,y0 ellipse centre coordinates
y0=100;
t=-pi:0.01:pi;
x=x0+a*cos(t);
y=y0+b*sin(t);
h4=plot(x,y,'Color','blue');

% Circle 
xunit = 25 * cos(t) + 225;
yunit = 25 * sin(t) + 150;
h5=plot(xunit, yunit,'Color','blue');

Obstacles=[h1 h2 h3 h4 h5];

startInObstacle = obstacleCheckRigid(Obstacles,start_node,r,c);
goalInObstacle = obstacleCheckRigid(Obstacles,goal_node,r,c);

if or(startInObstacle==1,or(or(start_node(1)>xmax,start_node(1)<0),or(start_node(2)>ymax,start_node(2)<0)))
    outside_obs_start=1;
    while or(outside_obs_start==1,or(or(start_node(1)>xmax,start_node(1)<0),or(start_node(2)>ymax,start_node(2)<0)))
        % Display message if start node falls outside of action space 
        fprintf('\n');
        disp('INAVLID START NODE LOCATION!');
        fprintf('\n');
        prompt = 'Enter new x,y STARTING node location (e.g. [0,0]), relative to bottom left corner of action space: ';
        start_node = input(prompt);
        outside_obs_start = obstacleCheckRigid(Obstacles,start_node,r,c);
    end
end

if or(goalInObstacle==1,or(or(goal_node(1)>xmax,goal_node(1)<0),or(goal_node(2)>ymax,goal_node(2)<0)))
    outside_obs_goal=1;
    while or(outside_obs_goal==1,or(or(goal_node(1)>xmax,goal_node(1)<0),or(goal_node(2)>ymax,goal_node(2)<0)))
        % Display message if goal node falls outside of action space 
        fprintf('\n');
        disp('INAVLID GOAL NODE LOCATION!');
        fprintf('\n');
        prompt = 'Enter new x,y GOAL node location (e.g. [0,0]), relative to bottom left corner of action space: ';
        goal_node = input(prompt);
        outside_obs_goal = obstacleCheckRigid(Obstacles,goal_node,r,c);
    end
end

% Start program run timer
tic

% Plot yellow circle around goal node, representing distance
% threshold/margin
th = 0:pi/50:2*pi;
x_circle = .5 * cos(th) + goal_node(1);
y_circle = .5 * sin(th) + goal_node(2);
plot(x_circle, y_circle);
fill(x_circle, y_circle, 'y');
drawnow

% Plot start and end point
plot(start_node(1), start_node(2),'mo','MarkerFaceColor','m');
plot(goal_node(1), goal_node(2),'m^','MarkerFaceColor','m');

uistack(fig,'top');

% Initialize start node info
Nodes(1).x=start_node(1);
Nodes(1).y=start_node(2);
Nodes(1).Explored=0;
Nodes(1).ParentID=0;
Nodes(1).Theta=start_node(3);
Nodes(1).ID=1;
Nodes(1).Cost2Come=0;
Nodes(1).Cost2Go=sqrt((abs(start_node(1)-goal_node(1))^2)+(abs(start_node(2)-goal_node(2))^2));
Nodes(1).TotalCost=Nodes(1).Cost2Go+Nodes(1).Cost2Come;
Explored=zeros(650,450,13);

% Convert angle to 0-360 scale
start_node(3)=wrapTo360(start_node(3));

% Assign index for explored node orientations
if start_node(3)==0
    angle_tag=1;
elseif start_node(3)==30
    angle_tag=2;
elseif start_node(3)==60
    angle_tag=3;
elseif start_node(3)==90
    angle_tag=4;
elseif start_node(3)==120
    angle_tag=5;
elseif start_node(3)==150
    angle_tag=6;
elseif start_node(3)==180
    angle_tag=7;
elseif start_node(3)==210
    angle_tag=8;
elseif start_node(3)==240
    angle_tag=9;
elseif start_node(3)==270
    angle_tag=10;
elseif start_node(3)==300
    angle_tag=11;
elseif start_node(3)==330
    angle_tag=12;
elseif start_node(3)==360
    angle_tag=13;
else
end

% Round off start and end node lcoations
if and((ceil(start_node(1))-start_node(1))<=0.5,(ceil(start_node(1))-start_node(1))~=0)==1
    start_node(1)=ceil(start_node(1))-0.5;
else
    start_node(1)=floor(start_node(1));
end
if and((ceil(start_node(2))-start_node(2))<=0.5,(ceil(start_node(2))-start_node(2))~=0)==1
    start_node(2)=ceil(start_node(2))-0.5;
else
    start_node(2)=floor(start_node(2));
end

startx=2*start_node(1)+1; starty=2*start_node(2)+1;
Explored(startx,starty,angle_tag)=1;

% Toggle that states goal node has not been explored
goal_node_explored=0;

% Initialize node ID counter and Parent ID counter
i=1;
ParentIdx=1;

% Tags that represent all 5 possible moves (inputs to changeAngle.m
%   function)
movementTags=[2 1 0 -1 -2];
n=length(movementTags);
Xs=[]; Ys=[];
xx=start_node(1); yy=start_node(2);
startAngle=start_node(3);
firstIteration=1;
T=[];
% While goal node HAS NOT BEEN EXPLORED, perform every possible action and
%   record info in running structure
while goal_node_explored==0
    cost_ref=1000000000;
    if firstIteration==1
        firstIteration=0;
        firstIteration1=1;
    else
        
        % Sort unexplored nodes by lowest cost2come+cost2go
        T = struct2table(Nodes); % convert the struct array to a table
        sortedT = sortrows(T, 'TotalCost'); % sort the table by 'TotalCost'
        toDelete = sortedT.Explored == 1;
        sortedT(toDelete,:) = [];
        sortedS = table2struct(sortedT); % change it back to struct array if necessary
        
        % Extract first node in the sorted queue
        xx=sortedS(1).x; yy=sortedS(1).y; 
        ParentIdx=sortedS(1).ID; 
        startAngle=sortedS(1).Theta;
        isExplored=sortedS(1).Explored;
        
    end
    for k=1:1:n
        % Generate new node
        [newX,newY,newTheta] = changeAngle(xx,yy,startAngle,theta,dist,movementTags(k));
        
        if firstIteration1==1
            isExplored=0;
            firstIteration1=0;
        else
        end

        % Round value of the new node's x and y values
        NEWX=newX; NEWY=newY;
        if (ceil(newX)-newX)<=0.2
            newX=ceil(newX);
        elseif and((ceil(newX)-newX)>0.2,(ceil(newX)-newX)<=0.6)
            newX=ceil(newX)-0.5;
        else
            newX=floor(newX);
        end
        
        if (ceil(newY)-newY)<=0.2
            newY=ceil(newY);
        elseif and((ceil(newY)-newY)>0.2,(ceil(newY)-newY)<=0.6)
            newY=ceil(newY)-0.5;
        else
            newY=floor(newY);
        end
        
        % Check if new node is in any of the obstacles
        outsideObstaclesANDBorder=obstacleCheckRigid(Obstacles,[NEWX,NEWY],r,c);
        Anglee=wrapTo360(newTheta);
        if Anglee==0
            angle_tag=1;
        elseif Anglee==30
            angle_tag=2;
        elseif Anglee==60
            angle_tag=3;
        elseif Anglee==90
            angle_tag=4;
        elseif Anglee==120
            angle_tag=5;
        elseif Anglee==150
            angle_tag=6;
        elseif Anglee==180
            angle_tag=7;
        elseif Anglee==210
            angle_tag=8;
        elseif Anglee==240
            angle_tag=9;
        elseif Anglee==270
            angle_tag=10;
        elseif Anglee==300
            angle_tag=11;
        elseif Anglee==330
            angle_tag=12;
        elseif Anglee==360
            angle_tag=13;
        else
        end
            
        matrix_X=newX*2+1; matrix_Y=newY*2+1; matrix_angle=angle_tag;
        
        % Check if new node is within 0.5 distance of any other node
        if isempty(Xs)==1
            num=0;
        else
            Xstmp=Xs-NEWX; Ystmp=Ys-NEWY;
            if dist~=1
                num=sum(and(abs(Xstmp)<=.5,abs(Ystmp)<=.5));
            else
                  num=sum(and(abs(Xstmp)<=.5,abs(Ystmp)<=.5));
            end
        end
        
        if and(and(newX>=0,newY>=0),num==0)    
            d = point_to_line([goal_node(1),goal_node(2),0],[xx,yy,0],[newX,newY,0]);
            xDist=abs(NEWX-xx); yDist=abs(NEWY-yy);
            xBetween=0;
            yBetween=0;
            if xDist>yDist
                xBetween=and(goal_node(1)>min([NEWX,xx]),goal_node(1)<max([NEWX,xx]));
            else
                yBetween=and(goal_node(2)>min([NEWY,yy]),goal_node(2)<max([NEWY,yy]));
            end
            closeEnough=and(or(xBetween==1,yBetween==1),d<=0.5);
            
            if and(and(outsideObstaclesANDBorder==0,or(closeEnough==1,and(and(newX>=0,newX<=xmax),and(newY>=0,newY<=ymax))))==1,sum(Explored(matrix_X,matrix_Y,:))==0)==1                
                 if dist<=5
                     cost2come=Nodes(ParentIdx).Cost2Come+dist;
                 else
                     cost2come=Nodes(ParentIdx).Cost2Come+dist;
                 end
                cost2go=sqrt((abs(NEWX-goal_node(1))^2)+(abs(NEWY-goal_node(2))^2));
                sumCost=cost2come+cost2go;

                if or(abs(cost2go)<=0.5, and(or(xBetween==1,yBetween==1),d<=0.5))==1
                %if abs(cost2go)<=0.5
                    goal_node_explored=1;
                else
                end

                i=i+1;
                if goal_node_explored==1
                    % Save new node in "Node" structure, only if its the goal node
                    drawnow
                    line([xx goal_node(1)],[yy goal_node(2)]);            
                    Nodes(i).x=goal_node(1);
                    Nodes(i).y=goal_node(2);
                    Nodes(i).ParentID=ParentIdx;
                    Nodes(i).ID=i;
                    Nodes(i).Theta=Anglee;
                    Nodes(i).Cost2Come=cost2come;
                    Nodes(i).Cost2Go=cost2go;
                    Nodes(i).TotalCost=sumCost;
                    Nodes(i).Explored=0;
                else
                    % Save new node in "Node" structure, only if its the goal node                    
                    drawnow
                    line([xx NEWX],[yy NEWY]);
                    Nodes(i).x=newX;
                    Nodes(i).y=newY;
                    Nodes(i).ParentID=ParentIdx;
                    Nodes(i).ID=i;
                    Nodes(i).Theta=Anglee;
                    Nodes(i).Cost2Come=cost2come;
                    Nodes(i).Cost2Go=cost2go;
                    Nodes(i).TotalCost=sumCost;
                    Nodes(i).Explored=0;
                    Xs=[Xs newX]; Ys=[Ys newY];
                end
            else
            end
        else
        end
    end
    
    Anglee=wrapTo360(startAngle);
    if Anglee==0
        angle_tag=1;
    elseif Anglee==30
        angle_tag=2;
    elseif Anglee==60
        angle_tag=3;
    elseif Anglee==90
        angle_tag=4;
    elseif Anglee==120
        angle_tag=5;
    elseif Anglee==150
        angle_tag=6;
    elseif Anglee==180
        angle_tag=7;
    elseif Anglee==210
        angle_tag=8;
    elseif Anglee==240
        angle_tag=9;
    elseif Anglee==270
        angle_tag=10;
    elseif Anglee==300
        angle_tag=11;
    elseif Anglee==330
        angle_tag=12;
    elseif Anglee==360
        angle_tag=13;
    else
    end
    
    % Mark node as "Explored"
    matrix_X=xx*2+1; matrix_Y=yy*2+1;
    Explored(matrix_X,matrix_Y,angle_tag)=1;
    Nodes(ParentIdx).Explored=1;
end

% Backtrack to find optimal path, and plot it with a red line
backTrackingFinished=0;
node_idx=Nodes(end).ID;
k=0;
while backTrackingFinished==0
    k=k+1;
    xVals(k)=Nodes(node_idx).x;
    yVals(k)=Nodes(node_idx).y;
    node_idx=Nodes(node_idx).ParentID;
    if and(xVals(k)==start_node(1),yVals(k)==start_node(2))
        backTrackingFinished=1;
    else
    end
end
plot(xVals,yVals,'r-','LineWidth',1);
drawnow
plot(start_node(1), start_node(2),'mo','MarkerFaceColor','m');
plot(goal_node(1), goal_node(2),'m^','MarkerFaceColor','m');
drawnow
uistack(fig,'top');

% End program run timer
toc
