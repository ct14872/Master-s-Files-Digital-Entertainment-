function J = comp_jacob(thetas, lens)
    %Jacobian Matrix elements (derivatives)
    %Manually produce derivatives
    
    j1 = -lens(1)*sind(thetas(1));
    j2 = -lens(2)*sind(thetas(1)+thetas(2));
    j3 = -lens(3)*sind(thetas(1)+thetas(2)+thetas(3));
    j4 = lens(1)*cosd(thetas(1));
    j5 = lens(2)*cosd(thetas(1)+thetas(2));
    j6 = lens(3)*cosd(thetas(1)+thetas(2)+thetas(3));

    % 2 x 3 Jacobian matrix formulation
    J = [(j1+j2+j3) (j2+j3) j3; (j4+j5+j6) (j5+j6) j6];
    
end