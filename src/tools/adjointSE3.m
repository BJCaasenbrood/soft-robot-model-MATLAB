function y = adjointSE3(R,p), 
y = zeros(6,6); 
y(1:3,1:3) = R; 
y(4:6,1:3) = skew(p)*R; 
y(4:6,4:6) = R; 
end

function y = skew(v)
y = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0] ;
end