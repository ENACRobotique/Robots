function drawFrame( P0, P1, P2, P3, name)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    vectarrow(P0, P1, 'r');
    vectarrow(P0, P2, 'g');
    vectarrow(P0, P3, 'b');
    text(P0(1),P0(2),P0(3),['   ' ...
    name],'HorizontalAlignment','left','FontSize',8);

end

