% This function ingests a node's location and angle, and generates a new
% node location and angle/orientation.
function [newX,newY,newTheta] = changeAngle(x,y,startAngle,theta,dist,movementTag)
    COSD=cosd(startAngle);
    SIND=sind(startAngle);
    newAngle=(movementTag*theta)+startAngle;
    newTheta=newAngle;
    COSD2=cosd(newAngle);
    SIND2=sind(newAngle);
    if and(COSD<1,SIND>0)==1
        if and(COSD2<1,SIND2>0)==1
            newX=x+(dist*cosd(newAngle));
            newY=y+(dist*sind(newAngle));
        elseif and(COSD2<0,SIND2<1)==1
            newX=x+(dist*cosd(newAngle));
            newY=y+(dist*sind(newAngle));
        elseif and(COSD2>0,SIND2>-1)==1
            newX=x+(dist*cosd(newAngle));
            newY=y+(dist*sind(newAngle));
        else
        end
    elseif and(COSD<0,SIND<1)==1
        if and(COSD2<0,SIND2<1)==1
            newX=x+(dist*cosd(newAngle));
            newY=y+(dist*sind(newAngle));
        elseif and(COSD2<1,SIND2>0)==1
            newX=x+(dist*cosd(newAngle));
            newY=y+(dist*sind(newAngle));
        elseif and(COSD2>-1,SIND2<0)==1
            newX=x+(dist*cosd(newAngle));
            newY=y+(dist*sind(newAngle));
        else
        end        
    elseif and(COSD>-1,SIND<0)==1
        if and(COSD2>-1,SIND2<0)==1
            newX=x+(dist*cosd(newAngle));
            newY=y+(dist*sind(newAngle));
        elseif and(COSD2<0,SIND2<1)==1
            newX=x+(dist*cosd(newAngle));
            newY=y+(dist*sind(newAngle));
        elseif and(COSD2>0,SIND2>-1)==1
            newX=x+(dist*cosd(newAngle));
            newY=y+(dist*sind(newAngle));
        else
        end           
    elseif and(COSD>0,SIND>-1)==1
        if and(COSD2>0,SIND2>-1)==1
            newX=x+(dist*cosd(newAngle));
            newY=y+(dist*sind(newAngle));
        elseif and(COSD2>-1,SIND2<0)==1
            newX=x+(dist*cosd(newAngle));
            newY=y+(dist*sind(newAngle));
        elseif and(COSD2<1,SIND2>0)==1
            newX=x+(dist*cosd(newAngle));
            newY=y+(dist*sind(newAngle));
        else
        end           
    else
    end
    format long
    if (1-(sqrt((newY-y)^2+(newX-x)^2)))>.001
        tmp=1;
    else
    end
end

