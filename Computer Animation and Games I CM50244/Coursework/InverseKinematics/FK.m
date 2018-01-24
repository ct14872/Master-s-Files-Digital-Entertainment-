function [x,y] = FK(lens, thetas)
    % Forward kinematics given the lengths and current angles
    x = lens(1)*cosd(thetas(1)) + lens(2)*cosd(thetas(1)+thetas(2)) + lens(3)*cosd(thetas(1)+thetas(2)+thetas(3));
    y = lens(1)*sind(thetas(1)) + lens(2)*sind(thetas(1)+thetas(2)) + lens(3)*sind(thetas(1)+thetas(2)+thetas(3));
end