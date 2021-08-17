function S = PieceWiseFunction2(x,l)

if (x < l(1)) 
    s1 = 1;
    s2 = 0;
else
    s1 = 0;
    s2 = 1;
end

S = kron([s1,s2],eye(3));

end