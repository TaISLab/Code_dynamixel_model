function jacob = fcn_jacob(j,k,xj,Rref_O5,theta)
    jacob=0;
    jacob=diff(Rref_O5*xj(j,:)',theta(k));
end