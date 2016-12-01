function C = Straight_line(yPos,r,Fx,Fy,cCont,xVel,yVel,miu,m,g)
IsContact = false;
k= 100;
if (yPos < r)
    
   IsContact = true;
    yDisp = yPos - r;
        FDispy = - k * yDisp;
        if (FDispy < 0)
            FDispy = 0;
        end
        Fy = Fy + FDispy;
%         FDampContx = - cCont * xVel;
        FDampConty = - cCont * yVel;
        if (FDampConty + FDispy < 0)
            FDampConty = 0 - FDispy;
        end
        
        %Fx = Fx + FDampContx;
%         Fy = Fy + FDampConty;
        
        FFrx = -sign(xVel) * miu * abs(m*g);
        Fx = Fx + FFrx;

C = [ Fx, Fy ];
else 
    C =[];
end    
return
end