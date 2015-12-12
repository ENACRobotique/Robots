function [  ] = drawFoV( T_2Pg, f, size, n_plane, pt_plane, color )
%drawFoV Draw the pyramidal shape of the field of view of the camera
%   Inputs:
%       _ T_2Pg: transition matrix from cam to playground
%       _ aper: vector which contains the horizontal and vertical aperture
%       angle of the camera
%       _ n_plane: A normal vector to a 3D plane
%       _ pt_plane: a point on the 3D plane
%   Output:
%       
    ptOc_C = [0 0 0];
    ptOc_Pg = T_2Pg*[ptOc_C 1]';
    ptCorner0 = [size(1)/2; size(2)/2; f];
    ptCorner1 = [ptCorner0(1); -ptCorner0(2); ptCorner0(3)];
    ptCorner2 = [-ptCorner0(1); -ptCorner0(2); ptCorner0(3)];
    ptCorner3 = [-ptCorner0(1); ptCorner0(2); ptCorner0(3)];
    
    ptCorner0_Pg = T_2Pg*[ptCorner0; 1];
    ptCorner1_Pg = T_2Pg*[ptCorner1; 1];
    ptCorner2_Pg = T_2Pg*[ptCorner2; 1];
    ptCorner3_Pg = T_2Pg*[ptCorner3; 1];
    
    Corners_Pg = [ptCorner0_Pg';
               ptCorner1_Pg';
               ptCorner2_Pg';
               ptCorner3_Pg'];
    resCorners_Pg = zeros(4,4);
    for i=1:4
        [resCorners_Pg(i, 1:3) check] = plane_line_intersect(n_plane', pt_plane', ptOc_Pg(1:3, 1), Corners_Pg(i, 1:3)');
        if check == 1 ||  check == 3
           line = [ptOc_Pg(1:3, 1)';  resCorners_Pg(i, 1:3)] ;
           plot3(line(:, 1), line(:, 2), line(:, 3), color);
        end
    end
    
    resCorners_Pg = [resCorners_Pg; resCorners_Pg(1, :)];
    plot3(resCorners_Pg(:, 1), resCorners_Pg(:, 2), resCorners_Pg(:, 3), color);
end

