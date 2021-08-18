function y = adjointSE3inv(R,p)
y = zeros(6,6); 
Rt = R.'; 
y(1:3,1:3) = Rt;
y(4:6,1:3) = Rt*skew(p).'; 
y(4:6,4:6) = Rt;
end

function y = skew(v)
y = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0] ;
end