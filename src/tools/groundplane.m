function groundplane(r)
V = [-r,-r,0;
      r,-r,0;
      r,r,0;
      -r,r,0];
  
F = [1,2,3,4];

patch('Vertices',V,'Faces',F,'Facecolor',gcol(1),...
    'Edgecolor',gcol(3),'linewidth',2);
end

