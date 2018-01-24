function draw_circle_boundaries(center,radius)
    ang = 0:0.01:2*pi; 
    xp = radius*cos(ang);
    yp = radius*sin(ang);
    circle_plot = plot(center(1) + xp,center(2) + yp,'-g', 'LineWidth',5);
    uistack(circle_plot,'bottom');
    hold on;
end