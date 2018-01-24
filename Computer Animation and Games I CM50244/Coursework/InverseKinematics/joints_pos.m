function [x,y] = joints_pos(ip,thetas, lens)
% This function is mainly used to compute the points of the joints. Another
% approach would be to use forward Kinematics for this but this seemed
% simpler
    x = zeros(size(thetas,1),size(thetas,2)+1);
    y = zeros(size(thetas,1),size(thetas,2)+1); 
    x(1) = ip(1);
    y(2) = ip(2);
    n_t = [thetas(1) (thetas(1)+thetas(2)) (thetas(1)+thetas(2)+thetas(3))];
    for i = 1:size(thetas,2)
        [nx,ny] = pol2cart(deg2rad(n_t(i)),lens(i));
        x(i+1) = x(i)+ nx;
        y(i+1) = y(i)+ ny;
    end
end