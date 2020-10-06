function gamma = fcn_gamma(j,n,OiOj,B0,R)
    gamma=zeros(3,3);    
    for i = j+1:n
        sum=OiOj(i,j)*B0(i)*R(:,:,1,j+1);
        gamma=gamma+sum;
    end
end
