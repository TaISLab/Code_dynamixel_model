function xi = fcn_xi(j,n,B1,R)

    xi=zeros(3,3);    
    for i = j:n
        sum=R(:,:,i+1,j+1)*B1(:,:,i)*R(:,:,1,i+1);
        xi=xi+sum;
    end
end
