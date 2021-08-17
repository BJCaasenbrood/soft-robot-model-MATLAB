function y = stepspace(d1, d2, st)

if nargin == 2
    st = 1e-3;
else
   % n = floor(double(n));
end

if ~isscalar(d1) || ~isscalar(st) || ~isscalar(d2)
    error(message('MATLAB:linspace:scalarInputs'));
end

N = ceil((d2 - d1)/st);
y = linspace(d1,d2,N);

%n1 = n-1;
%c = (d2 - d1).*(n1-1); %check intermediate value for appropriate treatment
% if isinf(c)
%     if isinf(d2 - d1) %opposite signs overflow
%         y = d1 + (d2./n1).*(0:n1) - (d1./n1).*(0:n1);
%     else
%         y = d1 + (0:n1).*((d2 - d1)./n1);
%     end
% else
%y = d1 + (0:n1).*st;
%end

% if ~isempty(y)
%     if st == 0
%         y(:) = d1;
%     else
%         y(1) = d1;
%         y(end) = d2;
%     end
% end
