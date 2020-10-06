function U = fcn_U(j,k,n,OiOj,B0,B1,R)
    xi=zeros(3,3);    
    gamma_t=zeros(3,3);    
    U=zeros(3,3);    
    for t = k:n-1
       
        for i = max(j,t+1):n
            sum_xi=R(:,:,i+1,j+1)*OiOj(i,j)*B1(:,:,i)*R(:,:,t+1,i+1);
            xi=xi+sum_xi;
        end
        
        for i = max(j+1,t+1):n
            sum_gamma=OiOj(i,j)*B0(i)*R(:,:,t+1,j+1);
            gamma_t=gamma_t+sum_gamma;
        end
        
        sum=(xi+gamma_t)*OiOj(t,t+1)*R(:,:,k+1,t+1);
        U=U+sum;
    end
end
