function E = Straight_line_1(xPos1,r1,Fx1,Fx2,cCont,xVel1,yVel1,miu,m1,g ,k)
XLinePos1 = 25;
IsContact = false;
 if (xPos1+r1 > XLinePos1)
     IsContact = true;
        xDisp1 = xPos1 - r1;
        FDispx1 = - k * xDisp1;
        if (FDispx1 < XLinePos1)
            FDispx1= XLinePos1;
        end
       Fx1 = Fx1 + FDispx1;
        FDampContx = - cCont * xVel1;
%         FDampConty = - cCont * yVel1;
        if (FDampContx + FDispx1 < 0)
            FDampContx = 0 - FDispx1;
        end
        
        %Fx = Fx + FDampContx;
        Fx1 = -Fx1 + FDampContx;
        
        % FFrx =-sign(yVel1) * miu * abs(m1*g);
%        Fy1 = Fy1 + FFrx;
      
        
    E = Fx1 ;
else 
    E =[];
end    
return
end