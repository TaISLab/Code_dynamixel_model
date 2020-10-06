function theta_actual = fcn_inter_entorno(fuerza,theta,Kesfera,ptos_contacto)

theta_actual=theta;

delta_x=[0,0];
delta_theta=[0,0];

delta_x(1)=fuerza(1)/Kesfera;
delta_x(2)=fuerza(2)/Kesfera;


delta_theta(1)=atan(delta_x(1)/ptos_contacto(1));
delta_theta(2)=atan(delta_x(2)/ptos_contacto(2));

theta_actual(1)=theta(1)+delta_theta(1);
theta_actual(2)=theta(2)+delta_theta(2);

theta_actual=[theta_actual(1) theta_actual(2) theta(3)];
end

