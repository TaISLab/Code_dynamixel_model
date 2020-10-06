%% Inicialización
clear all;
close all;

syms q2i qa;

g=9.8; % Gravedad

% Ejes de cada articulación
xj = [1 0 0; 0 1 0; 0 0 1];

% Masas de los eslabones
m = [9.05 13.6 5.06 3.2]*10^-3; % Kg
Mtot = sum(m); % Masa total de todos los eslabones

% Longitudes de los eslabones
l = [40 20 60 25 27.8]*10^-3; % m

% Ángulos conocidos
psi = 90*pi/180; % rad
gamma = 56*pi/180; % rad

% Distancias auxiliares
f=sqrt(l(4)^2+l(5)^2+2*l(4)*l(5)*cos(qa+gamma));
g=sqrt(l(1)^2+l(2)^2+2*l(1)*l(2)*cos(psi-q2i));

% Ángulo qi
q1i= asin((l(4)/f)*sin(qa+gamma))+asin((l(2)/g)*sin(psi-q2i))+acos((f^2+g^2-l(3)^2)/(2*f*g))-gamma;

% Ángulos auxiliares
alpha1=q1i+gamma;
alpha2=pi-psi+q2i;
alphaa=pi-qa-gamma;
alpha3=asin((l(1)*sin(alpha2)/g))+acos((g^2+l(3)^2-f^2)/(2*g*l(3)));
alpha4=acos((f^2+l(4)^2-l(5)^2)/(2*f*l(4)))+acos((l(3)^2+f^2-g^2)/(2*l(3)*f));

syms q1 q2 q3 q4 q5
theta=[q1,q2,q3,q4,q5,0,0]; % Los dos últimos valores indican la posición del sistema de la articulación O1 con respecto al sistema de referencia inercial

%% Posiciones relativas de cada sistema de referencia respecto al sistema de referencia inercial
Rref_O5x=l(1)*cos(q1)+l(2)*cos(q2)+l(3)*cos(q3)+l(4)*cos(q4);
Rref_O5y=l(1)*sin(q1)+l(2)*sin(q2)+l(3)*sin(q3)+l(4)*sin(q4);
Rref_O5=[Rref_O5x Rref_O5y 0];


%% Cálculo de variable B
B0 = [9.05 13.6 5.06 3.2]*10^-3; % Kg
B11= [0 0 0; 0 0 -(m(1)*l(1))/2; 0 (m(1)*l(1))/2 0]; % Kg*m
B12= [0 0 0; 0 0 -(m(2)*l(2))/2; 0 (m(2)*l(2))/2 0]; % Kg*m
B13= [0 0 0; 0 0 -(m(3)*l(3))/2; 0 (m(3)*l(3))/2 0]; % Kg*m
B14= [0 0 0; 0 0 -(m(4)*l(4))/2; 0 (m(4)*l(4))/2 0]; % Kg*m

B1=B11;
B1(:,:,2)=B12;
B1(:,:,3)=B13;
B1(:,:,4)=B14;

B21= [0 0 0; 0 (m(1)*(l(1)^2))/3 0; 0 0 (m(1)*(l(1)^2))/3]; % Kg*m^2
B22= [0 0 0; 0 (m(2)*(l(2)^2))/3 0; 0 0 (m(2)*(l(2)^2))/3]; % Kg*m^2
B23= [0 0 0; 0 (m(3)*(l(3)^2))/3 0; 0 0 (m(3)*(l(3)^2))/3]; % Kg*m^2
B24= [0 0 0; 0 (m(4)*(l(4)^2))/3 0; 0 0 (m(4)*(l(4)^2))/3]; % Kg*m^2

B2=B21;
B2(:,:,2)=B22;
B2(:,:,3)=B23;
B2(:,:,4)=B24;

B31= [625.77 106.81 0; 106.81 5892.47 0; 0 0 5647.41]*10^-9; % Kg*m^2
B32= [7282.20 -38.87 0; -38.87 1929.72 0; 0 0 8442.66]*10^-9; % Kg*m^2
B33= [625.77 106.81 0; 106.81 5892.47 0; 0 0 5647.41]*10^-9; % Kg*m^2
B34= [134.10 425.72 89; 425.72 5620.37 8.29; 89 8.29 5695.45]*10^-9; % Kg*m^2

B3=B31;
B3(:,:,2)=B32;
B3(:,:,3)=B33;
B3(:,:,4)=B34;



%% Cálculo de las matrices de rotación
% Rotaciones que se producen en el sentido contrario de las agujas
% del reloj ==> ángulos positivos.

R01=[cos(q1) -sin(q1) 0; sin(q1) cos(q1) 0; 0 0 1];
R12=[cos(-q2) -sin(-q2) 0; sin(-q2) cos(-q2) 0; 0 0 1];

R02=R01*R12;
R23=[cos(-q3) -sin(-q3) 0; sin(-q3) cos(-q3) 0; 0 0 1];

R03=R02*R23;
R13=R12*R23;
R34=[cos(-q4) -sin(-q4) 0; sin(-q4) cos(-q4) 0; 0 0 1];

R04=R03*R34;
R14=R13*R34;
R24=R23*R34;
R45=[cos(-q5) -sin(-q5) 0; sin(-q5) cos(-q5) 0; 0 0 1];

R05=R04*R45;
R15=R14*R45;
R25=R24*R45;
R35=R34*R45;

R10=inv(R01);
R20=inv(R02);
R30=inv(R03);
R40=inv(R04);
R50=inv(R05);
R21=inv(R12);
R32=inv(R23);
R31=inv(R13);
R43=inv(R34);
R41=inv(R14);
R42=inv(R24);
R54=inv(R45);
R51=inv(R15);
R52=inv(R25);
R53=inv(R35);

R0=sym([1 0 0; 0 1 0; 0 0 1]);
R0(:,:,2)=R01;
R0(:,:,3)=R02;
R0(:,:,4)=R03;
R0(:,:,5)=R04;
R0(:,:,6)=R05;


R1=R10;
R1(:,:,2)=sym([1 0 0; 0 1 0; 0 0 1]);
R1(:,:,3)=R12;
R1(:,:,4)=R13;
R1(:,:,5)=R14;
R1(:,:,6)=R15;

R2=R20;
R2(:,:,2)=R21;
R2(:,:,3)=sym([1 0 0; 0 1 0; 0 0 1]);
R2(:,:,4)=R23;
R2(:,:,5)=R24;
R2(:,:,6)=R25;

R3=R30;
R3(:,:,2)=R31;
R3(:,:,3)=R32;
R3(:,:,4)=sym([1 0 0; 0 1 0; 0 0 1]);
R3(:,:,5)=R34;
R3(:,:,6)=R35;

R4=R40;
R4(:,:,2)=R41;
R4(:,:,3)=R42;
R4(:,:,4)=R43;
R4(:,:,5)=sym([1 0 0; 0 1 0; 0 0 1]);
R4(:,:,6)=R45;

R5=R50;
R5(:,:,2)=R51;
R5(:,:,3)=R52;
R5(:,:,4)=R53;
R5(:,:,5)=R54;
R5(:,:,6)=sym([1 0 0; 0 1 0; 0 0 1]);

R=R0;
R(:,:,:,2)=R1;
R(:,:,:,3)=R2;
R(:,:,:,4)=R3;
R(:,:,:,5)=R4;
R(:,:,:,6)=R5; % Notación: Rjk=R(:,:,k,j)


%% Cálculo de las distancias entre las articulaciones
% Las distancias no directas se han obtenido mediante el teorema del coseno.
OiOj=sym(zeros(5,5));
OiOj(1,1)=0;
OiOj(1,2)=l(1);
OiOj(1,3)=g;
OiOj(1,4)=f;
OiOj(1,5)=l(5);

OiOj(2,1)=OiOj(1,2);
OiOj(2,2)=0;
OiOj(2,3)=l(2);
OiOj(2,4)=sqrt(l(2)^2+l(3)^2-2*l(2)*l(3)*cos(alpha3));
OiOj(2,5)=sqrt(l(1)^2+l(5)^2-2*l(1)*l(5)*cos(alpha1));

OiOj(3,1)=OiOj(1,3);
OiOj(3,2)=OiOj(2,3);
OiOj(3,4)=l(3);
OiOj(3,5)=sqrt(l(3)^2+l(4)^2-2*l(3)*l(4)*cos(alpha4));

OiOj(4,1)=OiOj(1,4);
OiOj(4,2)=OiOj(2,4);
OiOj(4,3)=OiOj(3,4);
OiOj(4,5)=l(4);

OiOj(5,1)=OiOj(1,5);
OiOj(5,2)=OiOj(2,5);
OiOj(5,3)=OiOj(3,5);
OiOj(5,4)=OiOj(4,5);




%% Bucles para el cálculo de la matrix de inercias Icf
n=4;
Icf=sym(zeros(n+4,n+4));

for j=1:n-1 
   for k=j:n-1
      A=xj(3,:)*fcn_sigma(j,k,n,B2,B3,R)*xj(3,:)';
      B=-xj(3,:)*fcn_psi(j,k,n,OiOj,B1,R)*xj(3,:)';
      C=-xj(3,:)*fcn_U(j,k,n,OiOj,B0,B1,R)*xj(3,:)';
      Icf(j,k) = A+B+C;
      Icf(k,j) = Icf(j,k);
   end
end

for j=1:n 
   for k=n
      if j~=n
          A=xj(3,:)*fcn_sigma(j,k,n,B2,B3,R)*xj(3,:)';
          B=-xj(3,:)*fcn_psi(j,k,n,OiOj,B1,R)*xj(3,:)';
          Icf(j,k) = A+B;
          Icf(k,j) = Icf(j,k);
      else
          Icf(j,k) = 1;
      end
   end
end

for j=1:n 
   for k=n+1:n+2
      if j~=n
          D=xj(3,:)*fcn_xi(j,n,B1,R)*xj(k-n,:)';
          E=xj(3,:)*fcn_gamma(j,n,OiOj,B0,R)*xj(k-n,:)';
          Icf(j,k) = D+E;
          Icf(k,j) = Icf(j,k);
      else
          Icf(j,k) = D;
          Icf(k,j) = Icf(j,k);
      end
   end
end

for j=n+1:n+2 
   for k=n+1:n+2
      if j==k
          Icf(j,k) = Mtot;
      else
          Icf(j,k) = 0;
      end
   end
end

for j=n+3:n+4 
   for k=1:n+2
      if (j~=n+3 && k~=n+1)|| (j~=n+4 && k~=n+2)
          Icf(j,k) = fcn_jacob(j-n-2,k,xj,Rref_O5,theta);
          Icf(k,j) = -Icf(j,k);
      else
          Icf(j,k) = 0;
          Icf(k,j) = Icf(j,k);
      end
   end
end

for j=n+3:n+4 
   for k=n+3:n+4
      Icf(j,k) = 0;
      Icf(k,j) = Icf(j,k);
   end
end


%% Término Rcf




ws=0;
for k = -1:i
    sum_ws=R(:,:,k+1,i+1)*xj(k,:)'*acel_art(k);
    ws = ws+sum_ws;
end


wv=0;
for k = -1:i-1
    sum_wv=cross(R(:,:,k+1,i+1)*[0;0;vector_theta_p(k)],R(:,:,k+2,i+1)*xj(3,:)'*acel_art(k+1));
    wv=wv+sum_wv;
end

w=ws+ws;

ros = 0;
ros2 = 0;
ws2=0;
for j = 1:3
    for k = -1:i-1
      for h = -1:k
            sum_ws=R(:,:,h+1,k+1)*xj(h,:)'*acel_art(h);
            ws2 = ws2+sum_ws;
      end
      sum_2_ros=R(:,:,k+1,i+1)*cross(ws2,OiOj(k+1,k));
      ros2=ros2+sum_2_ros;
    end
    sum_ros=R(:,:,1,i+1)*xj(j,:)'*acel_lineal(j)+ros2;
    ros=ros+sum_ros;
end

rov=0;
rov2=0;
for k=-1:i-1
    wv2=0;
    for h = -1:k-1
        sum_wv2=cross(R(:,:,h+1,k+1)*[0;0;vector_theta_p(k)],R(:,:,h+2,k+1)*xj(3,:)'*acel_art(h+1));
        wv2=wv2+sum_wv2;
    end
    sum_rov2=R(:,:,k+1,i+1)*(cross(wv2,OiOj(k+1,k))+cross([0;0;vector_theta_p(k)],(cross([0;0;vector_theta_p(k)],OiOj(k+1,k)))));
    rov2=rov2+sum_rov2;
end
rov=R(:,:,1,i+1)*xj(2,:)'*g+rov2;
