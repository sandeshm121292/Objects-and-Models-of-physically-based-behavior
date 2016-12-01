function main
    close all; clear all;

    m1 = 5; m2= 7; 
    k1 = 300; k2=200; 
    c2=30; c1 = 20;
    m = [m2 m1 m1 m2 m1];
    k = [k1, k1, k2, k2, k1, k1, k1];
    c = [c1, c1, c2, c2, c1, c1, c1]/10;
    coord = [3 4; 1 2; 3 1; 5 1; 5 2;  ];
    elm = [1 2; 2 3;  3 4; 3 5; 4 5; 1 3; 5 1];

    g = 9.8;
    F = [0 0  0 -m(2)*g  0 -m(3)*g   0 0  0 -m(5)*g]'; 
    
    IS = logical([ 1 1  1 0  0 0  1 1  0 0]);
    nNodes = length(m); nElm = length(k);
    
    DOF = 2;
    
    U = zeros(nNodes * DOF, 1);  % displacements
    render(coord, elm, IS, U);
    
    % Collecting inhibition of matrix (linear tasks)
    C=zeros(nNodes*DOF);
    
    for iii=1:nElm   %element cycle: formation of stiffness matrix
       i=elm(iii,1);j=elm(iii,2);
       Cel=[ 1 0 -1 0;
               0 0  0 0;
              -1 0  1 0;
               0 0  0 0]*c(iii);  % elemento matrica LKS
       alpha=atan2(coord(j,2)-coord(i,2),coord(j,1)-coord(i,1));
       % Coordinate conversion matrix:
       T=[ cos(alpha) sin(alpha)        0         0;
          -sin(alpha) cos(alpha)        0         0;
                0          0     cos(alpha) sin(alpha) ;
                0          0   -sin(alpha) cos(alpha) ];
        Ce=T'*Cel*T;   % elemento matrica GKS
        indK=[(i-1)*DOF+1,i*DOF,(j-1)*DOF+1,j*DOF];
        C(indK,indK)=C(indK,indK)+Ce; % Arrays are entered into the design matrix
     end %******************************* 
    % Collecting matrix M
    for iii=1:nNodes
       M(iii*DOF -1,iii*DOF -1) = m(iii); 
       M(iii*DOF,iii*DOF)       = m(iii); 
    end
    
    
    TT = 0.5; % Integration time (s)
    dt = 0.002; % Integravimo zingsnis (s)
    % numerical integration 
    U  = zeros(nNodes*DOF,1); % displacement vector
    DU = zeros(nNodes*DOF,1); % 1'oji displacement vector derivative according to the time - speed
    DDU = zeros(nNodes*DOF,1);% 2'oji the displacement vector derivative by time - acceleration
  
    nsteps=TT/dt;     Urez(1:nNodes * DOF,1:nsteps)=0;    i = 0; 
    colorsLines={'b-';'r-';'g-';'m-';'c-';'k-';'b-';'r-';'g-';'m-';'c-';'k-';}; 
    
    epsmax=1e-8; itmax=50;  % l.s. solution accuracy and the permissible number of iterations
    bet0=1/2;bet1=1/2;bet2=1; % Newmark method parameters        
  
    for t=0:dt:TT            % **********Numerical integration 
    
        deltaDDU=zeros(2*nNodes,1);    % acceleration gain
        q0=U+dt*DU+dt*dt/2*DDU;  
        b0=bet0*dt*dt/2; %initial DU DU DU estimates for the Taylor series
        q1=DU+dt*DDU; 
        b1=bet1*dt;
        q2=DDU; b2=1;
        
        for iii=1:itmax         % ----------Newton's method Rafsono each int. In step
            KT=jakobi(U,coord,elm,k);                   % statics task Jacobi matrix
            
            psi=unexpected(U,coord,elm,k,F);   %dynamic equilibrium would not be appropriate
            
            psi(find(~IS)) = psi(find(~IS)) - M(find(~IS), find(IS))*DDU(find(IS))-C(find(~IS), find(IS))*DU(find(IS)); 
            psi = psi - M*DDU-C*DU;
            
            KD=b2*M+b1*C+b0*KT;            % Jacobi matrix, to measure the dynamics of the members    
            INCR=zeros(2*nNodes,1);           % solution gains
            INCR(find(~IS))=KD(find(~IS),find(~IS))\(psi(find(~IS))); % 
            deltaDDU=deltaDDU+INCR;        % adjusted solution  
            eps=norm(INCR)/(norm(deltaDDU)+norm(INCR)); %The accuracy of the current in step
            U=q0+b0;              % U,DU,DDU specification
            DU=q1+b1*deltaDDU;
            DDU=q2+b2*deltaDDU; 
            if eps<epsmax, break, 
            end
        end                  % ---------- NR end of iterations
        
        render(coord, elm, IS, U);
        i = i+1;   Urez(:,i)=U; % displacement of preservation
        
        [DDU(1), DU(1), U(1) ] = two_time_function(1, 1, t);
   
        
    end
    
    % ***********Numerical integration of the end
    disp('after Numark: U = ');disp(U');
    render(coord, elm, IS, U);
    
    figure(2);hold on; grid on; 
    title('displacements X'); xlabel('time (s)'); ylabel('displacements (m)');
    for i=1:nNodes 
        plot([0:dt:TT],Urez((i-1) * DOF +1 ,:),colorsLines{i});
    end
    legend('1 node','2 node', '3 node', '4 node', '5 node');
    figure(3);hold on; grid on; 
    title('displacements Y'); xlabel('time (s)'); ylabel('displacements (m)');
    for i=1:nNodes 
        plot([0:dt:TT],Urez(i * DOF ,:),colorsLines{i});
    end
    legend('1 node','2 node', '3 node', '4 node', '5 node');
    figure(2);hold on;

    figure(4);hold on;
    plot([0:dt:TT],Urez(1,:),'--b');
    plot([0:dt:TT],Urez(2,:),'-b');
    plot([0:dt:TT],Urez(3,:),'--r');
    plot([0:dt:TT],Urez(4,:),'-r');
    plot([0:dt:TT],Urez(5,:),'--g');
    plot([0:dt:TT],Urez(6,:),'-g');

    figure(5);hold on;
    plot([0:dt:TT],Urez(1*DOF+2,:),'--b');
    plot([0:dt:TT],Urez(2*DOF+2,:),'-b');
    plot([0:dt:TT],Urez(3*DOF+2,:),'--r');
    plot([0:dt:TT],Urez(4*DOF+2,:),'-r');
    plot([0:dt:TT],Urez(5*DOF+2,:),'--g');
    plot([0:dt:TT],Urez(6*DOF+2,:),'-g');
end



function KT=jakobi(U,cor,ind,k)  % 2D spring assembly Jacobi matrix

    nel=size(ind,1);nmz=size(cor,1);
    KT=zeros(2*nmz,2*nmz); 
    for i=1:nel
       mzr=ind(i,1); mzs=ind(i,2); r=[2*mzr-1,2*mzr]; s=[2*mzs-1,2*mzs];
       cr=cor(mzr,:);cs=cor(mzs,:);
       Lvec=cs+U(s)'-cr-U(r)'; L=norm(Lvec); n=Lvec/L;
       L0=norm(cs-cr); 
       T=k(i)*(L-L0);
       table(1:2,1:2)=k(i)*[ 1-L0/L+Lvec(1)^2*L0/L^3, Lvec(1)*Lvec(2)*L0/L^3; ...
                           Lvec(1)*Lvec(2)*L0/L^3, (1-L0/L+Lvec(2)^2*L0/L^3) ];
       KT([r,s],[r,s])=KT([r,s],[r,s])+[table, -table; -table, table];
    end
return
end

function psi=unexpected(U,cor,ind,k,F)  % l.s. function value (unexpected)
    nel=size(ind,1);nmz=size(cor,1);
    psi=zeros(2*nmz,1);
    for i=1:nel     % Each item added to the force vector functions
       mzr=ind(i,1); mzs=ind(i,2); r=[2*mzr-1,2*mzr]; s=[2*mzs-1,2*mzs];
       cr=cor(mzr,:);cs=cor(mzs,:);
       Lvec=cs+U(s)'-cr-U(r)'; L=norm(Lvec); n=Lvec/L;
       L0=norm(cs-cr); 
       T=k(i)*(L-L0);
       psi([r,s])=psi([r,s])+[n'; -n']*T;
    end
    psi=psi+F;
return
end

% Structure mapping f-ja
function render(coord, elm, IS, U)
    [nElm xx] = size(elm);
    [nNodes DOF] = size(coord);
    r = 0.2; % particle radius in portraying
    ff = figure(1); clf(ff);
    hold on; grid on; axis([0, 7, 0, 6]);
    %display particles
    for i=1:nNodes
        u = U((i-1) * DOF +1);% ith particle displacement x direction
        v = U((i) * DOF);% ith particle displacement in the direction of y
        rectangle('Position', [coord(i,1)+u-r, coord(i,2)+v-r, 2*r, 2*r ], 'Curvature', [1, 1], 'FaceColor', [ 0.4 0.6 1 ]);
        iX = IS((i-1) * DOF +1); % Node consolidation of the x-direction
        iY = IS((i) * DOF); % Node consolidation of the x-direction
        if iX ~= 0, line([coord(i,1)+u coord(i,1)+u], [coord(i,2)+v-r coord(i,2)+v+r], 'Color',[ 0.2 0.2 0.2],'LineWidth',3); end
        if iY ~= 0, line([coord(i,1)+u-r coord(i,1)+u+r], [coord(i,2)+v coord(i,2)+v], 'Color',[ 0.2 0.2 0.2],'LineWidth',3); end
    end
    
    %Displays connections
    for i=1:nElm
        r = elm(i, 1); s = elm(i, 2); % r, s - no particles connecting the nodes
        ur = U((r-1) * DOF +1); vr = U((r) * DOF);
        us = U((s-1) * DOF +1); vs = U((s) * DOF);
        line([coord(r,1)+ur coord(s,1)+us], [coord(r,2)+vr coord(s,2)+vs], 'Color',[ 0.2 0.2 0.2],'LineWidth',1);
    end
end

function [ddu du u]=two_time_function(deltaU,U_deltaT, t)
    ddu =0; du =0;u =0;
    if  t <=  U_deltaT, %If the displacement is carried out
        if(U_deltaT > 0)
             % shift happening - Calculate node acceleration,
             % speed shift
             omega=(pi)/U_deltaT;
             ddu = (deltaU /2) * omega^2 * sin (omega *(t)+(3/2)*pi);
             du = (deltaU /2) * omega * cos(omega *(t)+(3/2)*pi);
             u = -(deltaU/2)  * omega *1.27* (sin(omega *(t)+(3/2)*pi)+1);
        else,         
            % node displacement = 0
            ddu =0;du =0;u =0;
        end
    else,
        % displacement has occurred = 0
        ddu = 0; du = 0; u = deltaU;
    end
    return
end