function J = jacobiano_t_inv(theta,ptos_contacto, l_falanges)

J=[1/ptos_contacto(1), -(ptos_contacto(2)+l_falanges(1)*cos(theta(2)))/(ptos_contacto(1)*ptos_contacto(2)); 0, 1/ptos_contacto(2)];

end

