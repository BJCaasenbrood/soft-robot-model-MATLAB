function y = adjointse3(Gamma,U)
Gs = skew(Gamma); Us = skew(U);
y = zeros(6,6);
y(1:3,1:3) = Gs; 
y(4:6,1:3) = Us; 
y(4:6,4:6) = Gs; 
end

function y = skew(v)
y = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0] ;
end