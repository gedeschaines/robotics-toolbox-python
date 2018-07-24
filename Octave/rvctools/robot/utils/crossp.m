function [v] = crossp(x, y)
   v = cross(reshape(x,1,3),reshape(y,1,3))';
end
