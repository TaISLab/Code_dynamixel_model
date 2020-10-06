function tau_a_transf = fcn_trasnformacion_taua(tau_a,l_eslabones,theta)

gamma=56*pi/180;

f=sqrt(l_eslabones(4)^2+l_eslabones(5)^2+2*l_eslabones(4)*l_eslabones(5)*cos(theta(3)+gamma));

alpha=acos((f^2+l_eslabones(4)^2-l_eslabones(5)^2)/(2*f*l_eslabones(4)));

tau_a_transf=(tau_a/l_eslabones(4))*cos(alpha)*f;
end

