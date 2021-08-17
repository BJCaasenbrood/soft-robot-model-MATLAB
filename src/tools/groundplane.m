function groundplane(r)
V = [-r,-r,0;
      r,-r,0;
      r,r,0;
      -r,r,0];
  
F = [1,2,3,4];

patch('Vertices',V,'Faces',F,'Facecolor',greycolors(1),...
    'Edgecolor',greycolors(3),'linewidth',2);
end

