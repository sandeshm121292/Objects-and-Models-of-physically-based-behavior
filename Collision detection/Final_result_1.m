clc; close all;
m1 = 0.1;
m2 = 0.05;
m3 = 0.05;

cAbs = 0;
cCont = 1;
k = 100;
miu = 0.025;
%% Radius
r1 = 2.5;
r2 = 1.0;
r3 = 1.5;
%% Ball 1
xPos1_0 = 3;
yPos1_0 = 14;
xVel1_0 = 3;
yVel1_0 = 0;

fiPos1_0 = 0;
fiVel1_0 = 0;
%% Ball 2
xPos2_0 = 10;
yPos2_0 = 10;
xVel2_0 = 1;
yVel2_0 = 0;

fiPos2_0 = 0;
fiVel2_0 = 0;
%% Ball 3
xPos3_0 = 17;
yPos3_0 = 8;
xVel3_0 = 0;
yVel3_0 = 0;

fiPos3_0 = 0;
fiVel3_0 = 0;
%% 
dtExp = 0.005;
TotalTime = 15;
%% 
I1 = m1 * r1^2 / 2;
I2 = m2 * r2^2 / 2;
I3 = m3 * r3^2 / 2;
% Given: U_0, U'_0
%% Ball 1
xPos1 = xPos1_0;
yPos1 = yPos1_0;
xVel1 = xVel1_0;
yVel1 = yVel1_0;

fiPos1 = fiPos1_0;
fiVel1 = fiVel1_0;
%% Ball 3
xPos3 = xPos3_0;
yPos3 = yPos3_0;
xVel3 = xVel3_0;
yVel3 = yVel3_0;

fiPos3 = fiPos3_0;
fiVel3 = fiVel3_0;
%% Ball 2
xPos2 = xPos2_0;
yPos2 = yPos2_0;
xVel2 = xVel2_0;
yVel2 = yVel2_0;

fiPos2 = fiPos2_0;
fiVel2 = fiVel2_0;
%% 
%XExpArr = [xPos];
%tExpArr = [0];
%% 
mx1=-0.5;
c1=10;
mx2=-0.5;
c2=10;
mx3=-0.5;
c3=10;

g = -9.81;

figure;
for t = dtExp:dtExp:TotalTime
    
    %% Ball 1
    Fx1 = 0;
    Fy1 = m1 * g;
    Ffi1 = 0;
    %% Ball 2
    Fx2 = 0;
    Fy2 = m2 * g;
    Ffi2 = 0;
    %% Ball 3
     Fx3 = 0;
    Fy3 = m3 * g;
    Ffi3 = 0;
    %% Ball 1
    if(xPos1>20)
        mx1=0;
        c1=0;
    else mx1= -0.5;
         c1=10;
    end
    %% Ball 2
    if(xPos2>20)
        mx2=0;
        c2=0;
    else mx2=-0.5;
         c2=10;
    end
    %% Ball 3
    if(xPos3>20)
        mx3=0;
        c3=0;
    else mx3=-0.5;
         c3=10;
    end
    %% 
           
    yLinePos1=((mx1*xPos1)+c1);
    yLinePos2=((mx2*xPos2)+c2);
    yLinePos3=((mx3*xPos3)+c3);
    XLinePos1=30;
    XLinePos2=30;
    XLinePos3=30;
    %% Ball 1
    if (yPos1 < yLinePos1+r1)
        yDisp1 = yPos1 - r1;
        FDispy1 = - k * yDisp1;
        if (FDispy1 < yLinePos1+r1)
            FDispy1= yLinePos1+r1;
        end
        Fy1 = Fy1 + FDispy1;
        FDampContx = - cCont * xVel1;
        FDampConty = - cCont * yVel1;
        if (FDampConty + FDispy1 < 0)
            FDampConty = 0 - FDispy1;
        end
        
        %Fx = Fx + FDampContx;
        Fy1 = Fy1 + FDampConty;
        
        FFrx =-sign(xVel1) * miu * abs(m1*g*sin(30));
        Fx1 = Fx1 + FFrx;
    end
    %% Collision with wall ball 1
    if (xPos1+r1 > XLinePos1)
        xDisp1 = xPos1 - r1;
        FDispx1 = - k * xDisp1;
        if (FDispx1 < XLinePos1)
            FDispx1= XLinePos1;
        end
       Fx1 = Fx1 + FDispx1;
%         FDampContx = - cCont * xVel1;
        FDampConty = - cCont * yVel1;
        if (FDampContx + FDispx1 < 0)
            FDampContx = 0 - FDispx1;
        end
        
        %Fx = Fx + FDampContx;
        Fx1 = -Fx1 + FDampContx;
        
         FFrx =-sign(yVel1) * miu * abs(m1*g);
        Fy1 = Fy1 + FFrx;
        
    end
    
   
    %% Ball 2
    if (yPos2 < yLinePos2+r2)
        yDisp2 = yPos2 - r2;
        FDispy2 = - k * yDisp2;
        if (FDispy2 < yLinePos2+r2)
            FDispy2 = yLinePos2+r2;
        end
        Fy2 = Fy2 + FDispy2;
        FDampContx2 = - cCont * xVel2;
        FDampConty2 = - cCont * yVel2;
        if (FDampConty2 + FDispy2 < 0)
            FDampConty2 = 0 - FDispy2;
        end
        
        %Fx = Fx + FDampContx;
        Fy2 = Fy2 + FDampConty2;
        
        FFrx1 = -sign(xVel2) * miu * abs(m2*g);
        Fx2 = Fx2 + FFrx1;
    end
 %% Collision with wall  ball 2
  if (xPos2+r2 > XLinePos2)
        xDisp2 = xPos2 - r2;
        FDispx2 = - k * xDisp2;
        if (FDispx2 < XLinePos2)
            FDispx2= XLinePos2;
        end
       Fx2 = Fx2 + FDispx2;
%         FDampContx = - cCont * xVel1;
%         FDampConty = - cCont * yVel1;
        if (FDampContx + FDispx2 < 0)
            FDampContx = 0 - FDispx2;
        end
        
%         Fx1 = -Fx1 + FDampContx;
        Fx2 = -Fx2 + FDampContx;
        
         FFrx =-sign(yVel2) * miu * abs(m2*g);
        Fy2 = Fy2 + FFrx;
       
  end 
  
  %%  
   %% Ball 3
    if (yPos3 < yLinePos3+r3)
        yDisp3 = yPos3 - r3;
        FDispy3 = - k * yDisp3;
        if (FDispy3 < yLinePos3+r3)
            FDispy3 = yLinePos3+r3;
        end
        Fy3 = Fy3 + FDispy3;
        FDampContx3 = - cCont * xVel3;
        FDampConty3 = - cCont * yVel3;
        if (FDampConty3 + FDispy3 < 0)
            FDampConty3 = 0 - FDispy3;
        end
        
        %Fx = Fx + FDampContx;
        Fy3 = Fy3 + FDampConty3;
        
        FFrx3 = -sign(xVel3) * miu * abs(m3*g);
        Fx3 = Fx3 + FFrx3;
    end
 %% Collision with wall ball 3
  if (xPos3+r3 > XLinePos3)
        xDisp3 = xPos3 - r3;
        FDispx3 = - k * xDisp3;
        if (FDispx3 < XLinePos3)
            FDispx3= XLinePos3;
        end
       Fx3 = Fx3 + FDispx3;
%         FDampContx = - cCont * xVel1;
%         FDampConty = - cCont * yVel1;
        if (FDampContx3 + FDispx3 < 0)
            FDampContx3 = 0 - FDispx3;
        end
        
%         Fx1 = -Fx1 + FDampContx;
        Fx3 = -Fx3 + FDampContx3;
        
         FFrx3 =-sign(yVel3) * miu * abs(m3*g);
        Fy3 = Fy3 + FFrx3;
        
  end 
  
   %%  Collision 1 to 2
    
    DistCent_1_2 = sqrt((xPos1 - xPos2)^2 + (yPos1 - yPos2)^2);
    DistInside_1_2 = r1 + r2 - DistCent_1_2;
    
    if (DistCent_1_2 < r1 + r2)
        %Ball 1
        xNormDirVect_1_2 = xPos1 - xPos2;
        yNormDirVect_1_2 = yPos1 - yPos2;
        %Ball 2
        xNormDirVect_1_2 = xNormDirVect_1_2 / DistCent_1_2;
        yNormDirVect_1_2 = yNormDirVect_1_2 / DistCent_1_2;
        
        xTangDirVect_1_2 = xNormDirVect_1_2 * cos(pi/2) + yNormDirVect_1_2 * sin(pi/2);
        yTangDirVect_1_2 = -xNormDirVect_1_2 * sin(pi/2) + yNormDirVect_1_2 * cos(pi/2);
        
        Fk = - k * DistInside_1_2;
        
        Fkx1 = -Fk * xNormDirVect_1_2;
        Fky1 = -Fk * yNormDirVect_1_2;
        Fkx2 = Fk * xNormDirVect_1_2;
        Fky2 = Fk * yNormDirVect_1_2;
        
        Fx1 = Fx1 + Fkx1;
        Fy1 = Fy1 + Fky1;
        Fx2 = Fx2 + Fkx2;
        Fy2 = Fy2 + Fky2;
        FFr_1_2 = Fk * miu;
        
        xVel1Rel2 = xVel1 - xVel2;
        yVel1Rel2 = yVel1 - yVel2;
        xVel2Rel1 = xVel2 - xVel1;
        yVel2Rel1 = yVel2 - yVel1;
        
        tanVel1Rel2 = xVel1Rel2 * xTangDirVect_1_2 + yVel1Rel2 * yTangDirVect_1_2;
        tanVel2Rel1 = xVel2Rel1 * xTangDirVect_1_2 + yVel2Rel1 * yTangDirVect_1_2;
        
        FFr1 = FFr_1_2 * r1 * sign(tanVel1Rel2);
        FFr2 = FFr_1_2 * r2 * sign(tanVel2Rel1);
        
        Ffi1 = Ffi1 + FFr1;
        Ffi2 = Ffi2 + FFr2;
    end
    %% Collision 2 to 3
    DistCent_2_3 = sqrt((xPos2 - xPos3)^2 + (yPos2 - yPos3)^2);
    DistInside_2_3 = r2 + r3 - DistCent_2_3;
    if (DistCent_2_3 < r2 + r3)
        %Ball 1
        xNormDirVect_2_3 = xPos2 - xPos3;
        yNormDirVect_2_3 = yPos2 - yPos3;
        %Ball 2
        xNormDirVect_2_3 = xNormDirVect_2_3 / DistCent_2_3;
        yNormDirVect_2_3 = yNormDirVect_2_3 / DistCent_2_3;
        
        xTangDirVect_2_3 = xNormDirVect_2_3 * cos(pi/2) + yNormDirVect_2_3 * sin(pi/2);
        yTangDirVect_2_3 = -xNormDirVect_2_3 * sin(pi/2) + yNormDirVect_2_3 * cos(pi/2);
        
        Fk1 = - k * DistInside_2_3;
        
        Fkx2 = -Fk1 * xNormDirVect_2_3;
        Fky2 = -Fk1 * yNormDirVect_2_3;
        Fkx3 = Fk1 * xNormDirVect_2_3;
        Fky3 = Fk1 * yNormDirVect_2_3;
        
        Fx2 = Fx2 + Fkx2;
        Fy2 = Fy2 + Fky2;
        Fx3 = Fx3 + Fkx3;
        Fy3 = Fy3 + Fky3;
        FFr_2_3 = Fk1 * miu;
        
        xVel1Rel3 = xVel2 - xVel3;
        yVel1Rel3 = yVel2 - yVel3;
        xVel2Rel2 = xVel3 - xVel2;
        yVel2Rel2 = yVel3 - yVel2;
        
        tanVel1Rel3 = xVel1Rel3 * xTangDirVect_2_3 + yVel1Rel3 * yTangDirVect_2_3;
        tanVel2Rel2 = xVel2Rel2 * xTangDirVect_2_3 + yVel2Rel2 * yTangDirVect_2_3;
        
        FFr2 = FFr_2_3 * r2 * sign(tanVel1Rel3);
        FFr3 = FFr_2_3 * r3 * sign(tanVel2Rel2);
        
        Ffi2 = Ffi2 + FFr2;
        Ffi3 = Ffi3 + FFr3;
    end
    %% Collision 1 to 3
    
     %% Collision 2 to 3
    DistCent_1_3 = sqrt((xPos1 - xPos3)^2 + (yPos1 - yPos3)^2);
    DistInside_1_3 = r1 + r3 - DistCent_1_3;
    if (DistCent_1_3 < r1 + r3)
        %Ball 1
        xNormDirVect_1_3 = xPos1 - xPos3;
        yNormDirVect_1_3 = yPos1 - yPos3;
        %Ball 2
        xNormDirVect_1_3 = xNormDirVect_1_3 / DistCent_1_3;
        yNormDirVect_1_3 = yNormDirVect_1_3 / DistCent_1_3;
        
        xTangDirVect_1_3 = xNormDirVect_1_3 * cos(pi/2) + yNormDirVect_1_3 * sin(pi/2);
        yTangDirVect_1_3 = -xNormDirVect_1_3 * sin(pi/2) + yNormDirVect_1_3 * cos(pi/2);
        
        Fk2 = - k * DistInside_1_3;
        
        Fkx1 = -Fk2 * xNormDirVect_1_3;
        Fky1 = -Fk2 * yNormDirVect_1_3;
        Fkx3 = Fk2 * xNormDirVect_1_3;
        Fky3 = Fk2 * yNormDirVect_1_3;
        
        Fx1 = Fx1 + Fkx1;
        Fy1 = Fy1 + Fky1;
        Fx3 = Fx3 + Fkx3;
        Fy3 = Fy3 + Fky3;
        FFr_1_3 = Fk2 * miu;
        
        xVel1Rel3 = xVel1 - xVel3;
        yVel1Rel3 = yVel1 - yVel3;
        xVel2Rel1 = xVel3 - xVel1;
        yVel2Rel1 = yVel3 - yVel1;
        
        tanVel1Rel3 = xVel1Rel3 * xTangDirVect_1_3 + yVel1Rel3 * yTangDirVect_1_3;
        tanVel2Rel1 = xVel2Rel1 * xTangDirVect_1_3 + yVel2Rel1 * yTangDirVect_1_3;
        
        FFr1 = FFr_1_3 * r1 * sign(tanVel1Rel3);
        FFr3 = FFr_1_3 * r3 * sign(tanVel2Rel1);
        
        Ffi1 = Ffi1 + FFr1;
        Ffi3 = Ffi3 + FFr3;
    end
    
    %%

    Fx1 = Fx1 - cAbs * xVel1;
%     Fx2 = Fx2 - cAbs * xVel2;
%     Fx3 = Fx3 - cAbs * xVel3;
    Fy1 = Fy1 - cAbs * yVel1;
%     Fy2 = Fy2 - cAbs * yVel2;
%     Fy3 = Fy3 - cAbs * yVel3;
    
    %% Ball 1
    % U'' = M^-1 * (F - C*U' - K*U)
    xAcc1 = 1/m1 * Fx1;
    xVel1 = xVel1 + dtExp * xAcc1;
    xPos1 = xPos1 + dtExp * xVel1;
    
    yAcc1 = 1/m1 * Fy1;
    yVel1 = yVel1 + dtExp * yAcc1;
    yPos1 = yPos1 + dtExp * yVel1;
    
    fiAcc1 = 1/I1 * Ffi1;
    fiVel1 = fiVel1 + dtExp * fiAcc1;
    fiPos1 = fiPos1 + dtExp * fiVel1;
    
    %% Ball 2
    xAcc2 = 1/m2 * Fx2;
    xVel2 = xVel2 + dtExp * xAcc2;
    xPos2 = xPos2 + dtExp * xVel2;
    
    yAcc2 = 1/m2 * Fy2;
    yVel2 = yVel2 + dtExp * yAcc2;
    yPos2 = yPos2 + dtExp * yVel2;
    
    fiAcc2 = 1/I2 * Ffi2;
    fiVel2 = fiVel2 + dtExp * fiAcc2;
    fiPos2 = fiPos2 + dtExp * fiVel2;

    %%
    %% Ball 3
    xAcc3 = 1/m3 * Fx3;
    xVel3 = xVel3 + dtExp * xAcc3;
    xPos3 = xPos3 + dtExp * xVel3;
    
    yAcc3 = 1/m3 * Fy3;
    yVel3 = yVel3 + dtExp * yAcc3;
    yPos3 = yPos3 + dtExp * yVel3;
    
    fiAcc3 = 1/I3 * Ffi3;
    fiVel3 = fiVel3 + dtExp * fiAcc3;
    fiPos3 = fiPos3 + dtExp * fiVel3;
    %% 
%     XExpArr = [XExpArr xPos];
%     tExpArr = [tExpArr, t];
    

    plot([0, 20], [10, 0], 'b-');
    hold on;
    plot([20,30], [0,0]);
    plot([30,30], [0, 20]);
    tCircl = 0:(pi/32):(2*pi);
    %% Circle 1
    xCircl = xPos1 + r1 * cos(tCircl);
    yCircl = yPos1 + r1 * sin(tCircl);
    
    xCirclRad1 = xPos1 + r1 * sin(fiPos1);
    yCirclRad1 = yPos1 + r1 * cos(fiPos1);
   
    plot(xCircl, yCircl, 'r-');
    plot([xPos1, xCirclRad1], [yPos1, yCirclRad1], 'b-');
    %% Circle 2
    xCircl1 = xPos2 + r2 * cos(tCircl);
    yCircl1 = yPos2 + r2 * sin(tCircl);
    
    xCirclRad2 = xPos2 + r2 * sin(fiPos2);
    yCirclRad2 = yPos2 + r2 * cos(fiPos2);
    
    plot(xCircl1, yCircl1, 'b-');
    plot([xPos2, xCirclRad2], [yPos2, yCirclRad2], 'b-');
     %% Circle 3
    xCircl3 = xPos3 + r3 * cos(tCircl);
    yCircl3 = yPos3 + r3 * sin(tCircl);
    
    xCirclRad3 = xPos3 + r3 * sin(fiPos3);
    yCirclRad3 = yPos3 + r3 * cos(fiPos3);
    
    plot(xCircl3, yCircl3, 'g');
    plot([xPos3, xCirclRad3], [yPos3, yCirclRad3], 'b-');
     %%
    hold off;
    axis([0, 30, -5, 15]);
    grid on;
    pause(0.01);
end
% 
% tExpArr;
% XExpArr;
% 
% figure;
% plot(tExpArr, XExpArr);
% grid on;