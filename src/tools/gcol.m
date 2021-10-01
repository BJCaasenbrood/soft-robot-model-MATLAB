function newcolor = greycolors(n)
if nargin < 1
newcolor = [0.85; 0.7; 0.4;0.2].*ones(4,3);
else
    X = [0.85; 0.7; 0.4;0.2].*ones(4,3);
    newcolor = X(n,:);
end
end

