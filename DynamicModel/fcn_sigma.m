function sigma = fcn_sigma(j,k,n,B2,B3,R)
    sigma=zeros(3,3);    
    for i = max(j,k):n
        sum=R(:,:,i+1,j+1)*(B2(:,:,i)+B3(:,:,i))*R(:,:,k+1,i+1);
        sigma=sigma+sum;
    end
end

