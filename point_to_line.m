% This function finds the distance from a point to a line
%   segment(normal/90-degree distance).  This is used to check if a node is
%   within the distance threshold from the goal node.
function d = point_to_line(pt, v1, v2)
      a = v1 - v2;
      b = pt - v2;
      d = norm(cross(a,b)) / norm(a);
end