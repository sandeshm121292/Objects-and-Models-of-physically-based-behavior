       function Fmat=linecontact(mx1,c1,xPos1,yPos1,r1,Fx1,Fy1,Ffi1,fiVel1,xVel1,yVel1)
       k=100;
       
       miu = 0.025;
       cContFr = 0.5;
       cCont = 5;
        IsContact = false;
        DistCent_1_2 = r1/2;
        DistInside_1_2 = r1 - DistCent_1_2;
      
        yLinePos=((mx1*xPos1)+c1);
        
        if (yPos1 < yLinePos+r1)
            IsContact = true;
            xCont = xPos1 - r1*cos(pi/4);
            yCont = yPos1 - r1*sin(pi/4);
        
            xNormDirVect_1_2 = xPos1 - xCont;
            yNormDirVect_1_2 = yPos1 - yCont;
            xNormDirVect_1_2 = xNormDirVect_1_2 / DistCent_1_2;
            yNormDirVect_1_2 = yNormDirVect_1_2 / DistCent_1_2;
        
            xTangDirVect_1_2 = xNormDirVect_1_2 * cos(pi/2) + yNormDirVect_1_2 * sin(pi/2);
            yTangDirVect_1_2 = -xNormDirVect_1_2 * sin(pi/2) + yNormDirVect_1_2 * cos(pi/2);
        
            Fk = - k * DistInside_1_2;
            Fkx1 = -Fk * xNormDirVect_1_2;
            Fky1 = -Fk * yNormDirVect_1_2;
        

            Fx1 = Fx1 + Fkx1;
            Fy1 = Fy1 + Fky1;
        


            FFr_1_2 = -Fk * miu;
        
            s1x = xCont - xPos1;
            s1y = yCont - yPos1;
        

            Cont1RotVelVect = cross([0, 0, fiVel1], [s1x, s1y, 0]);
        
        
            Cont1RotVelx = Cont1RotVelVect(1);
            Cont1RotVely = Cont1RotVelVect(2);
        
        
            xVel1Rel2 = (xVel1 + Cont1RotVelx);
            yVel1Rel2 = (yVel1 + Cont1RotVely);
        
        
            tanVel1Rel2 = xVel1Rel2 * xTangDirVect_1_2 + yVel1Rel2 * yTangDirVect_1_2;
        
        
            FFr1 = -tanVel1Rel2 * cContFr;
        
        
            if (abs(FFr_1_2) < abs(FFr1))
                FFr1 = sign(FFr1) * abs(FFr_1_2);
            end
        
        
        
               
            FFr1x = FFr1 * xTangDirVect_1_2;
            FFr1y = FFr1 * yTangDirVect_1_2;
       
        
            Fx1 = Fx1 + FFr1x;
            Fy1 = Fy1 + FFr1y;
       
        
            FFr1fi = cross([FFr1x, FFr1y, 0], [s1x, s1y, 0]);
        
        
            Ffi1 = Ffi1 + FFr1fi(3);
       
            normVel1Rel2 = xVel1Rel2 * xNormDirVect_1_2 + yVel1Rel2 * yNormDirVect_1_2;
        
        
            FElC1 = -normVel1Rel2 * cCont;
       
        
            FElC1x = FElC1 * xNormDirVect_1_2;
            FElC1y = FElC1 * yNormDirVect_1_2;
        
        
            Fx1 = Fx1 + FElC1x;
            Fy1 = Fy1 + FElC1y;
            
            Fmat = [Fx1,Fy1,Ffi1];
            
       
       
        else
            Fmat=[];
       end
        return
       end