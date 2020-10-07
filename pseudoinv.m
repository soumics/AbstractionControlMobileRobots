figure
sphere
axis equal

figure
[x,y,z] = sphere
surf(x,y,z)  % sphere centered at origin
hold on
surf(x+3,y-2,z)  % sphere centered at (3,-2,0)
surf(x,y+1,z-3)  % sphere centered at (0,1,-3)
%daspect([1 1 1])
