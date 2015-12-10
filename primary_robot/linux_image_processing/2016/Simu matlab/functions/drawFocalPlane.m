function [ Corners ] = drawFocalPlane( T_2Pg, aper, n_plane_C, pt_plane_C, color )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    ptOc_C = [0 0 0];
    ptOc_Pg = T_2Pg*[ptOc_C 1]';
    ptC = [0 0 100];
    ptCorner0 = rot(aper(2)/2, aper(1)/2, 0)*ptC';
    ptCorner1 = [ptCorner0(1); -ptCorner0(2); ptCorner0(3)];
    ptCorner2 = [-ptCorner0(1); -ptCorner0(2); ptCorner0(3)];
    ptCorner3 = [-ptCorner0(1); ptCorner0(2); ptCorner0(3)];
    
    Corners = [ptCorner0';
               ptCorner1';
               ptCorner2';
               ptCorner3'];
              
   Corners_Pg = zeros(4,4);
    for i=1:4
        [Corners(i, 1:3) check] = plane_line_intersect(n_plane_C', pt_plane_C', ptOc_C', Corners(i, 1:3)');
        if check == 1 ||  check == 3
            Corners_Pg(i, :) = (T_2Pg*[Corners(i, :), 1]')';

        end
    end
    
    Corners_Pg = [Corners_Pg; Corners_Pg(1, :)];
    
    plot3(Corners_Pg(:, 1), Corners_Pg(:, 2), Corners_Pg(:, 3), color);
    
end

