function psi_k = fcn_psi(j,k,n,OiOj,B1,R)
    psi_k=zeros(3,3);    
    for i = max(j+1,k):n
        sum=OiOj(i,j)*R(:,:,i+1,j+1)*B1(:,:,i)*R(:,:,k+1,i+1);
        psi_k=psi_k+sum;
    end
end

