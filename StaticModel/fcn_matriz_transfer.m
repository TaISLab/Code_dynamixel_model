function Matriz_T_inv = fcn_matriz_transfer(theta,l_eslabones,l_falanges)

gamma=56*pi/180;
psi=90*pi/180;

f=sqrt(l_eslabones(4)^2+l_eslabones(5)^2+2*l_eslabones(4)*l_eslabones(5)*cos(theta(3)+gamma));
M=-l_falanges(1)*(l_falanges(1)+2*l_eslabones(2)*cos(theta(2)-psi))+f^2-l_eslabones(3)^2-l_eslabones(2)^2;
N=l_falanges(1)*(l_falanges(1)+2*l_eslabones(2)*cos(theta(2)-psi))-f^2-l_eslabones(3)^2+l_eslabones(2)^2;

cot_beta=(l_eslabones(2)*sin(theta(2)-psi)*sqrt(4*(f^2)*l_eslabones(3)^2- N^2)+M*(l_falanges(1)+l_eslabones(2)*cos(theta(2)-psi)))/...
    -((l_falanges(1)+l_eslabones(2)*cos(theta(2)-psi))*sqrt(4*(f^2)*l_eslabones(3)^2- N)+M*l_eslabones(2)*sin(theta(2)-psi));

h=l_eslabones(2)*(cos(theta(2)-psi)-sin(theta(2)-psi)*cot_beta);

Matriz_T=[1, -((h)/(h+l_falanges(1))); 0, 1];
Matriz_T_inv=(inv(Matriz_T))';
end

