function t = fcn_matriz_t(theta,tau_a, Kmuelle)

t2=Kmuelle*theta(2)*tau_a;
t=[tau_a,t2]';
end

