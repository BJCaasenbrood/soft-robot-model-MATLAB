function S = PieceWiseFunction3(x,l)

if (x < l(1)) 
    s1 = 1;
    s2 = 0;
    s3 = 0;
elseif (x >= l(1)) && (x < (l(1)+l(2)))      
    s1 = 0;
    s2 = 1;
    s3 = 0;
else
    s1 = 0;
    s2 = 0;
    s3 = 1;
end

S = kron([s1,s2,s3],eye(3));

end