function S = PieceWiseFunction5(x,l)

A = cumsum(l);

if (x < A(1)) 
    s1 = 1;
    s2 = 0;
    s3 = 0;
    s4 = 0;
    s5 = 0;
elseif (x >= A(1)) && (x < A(2))      
    s1 = 0;
    s2 = 1;
    s3 = 0;
    s4 = 0;
    s5 = 0;
elseif (x >= A(2)) && (x < A(3))      
    s1 = 0;
    s2 = 0;
    s3 = 1;    
    s4 = 0;
    s5 = 0;
elseif (x >= A(3)) && (x < A(4))      
    s1 = 0;
    s2 = 0;
    s3 = 0;    
    s4 = 1;    
    s5 = 0;
else
    s1 = 0;
    s2 = 0;
    s3 = 0;
    s4 = 0;
    s5 = 1;
end

S = kron([s1,s2,s3,s4,s5],eye(3));

end