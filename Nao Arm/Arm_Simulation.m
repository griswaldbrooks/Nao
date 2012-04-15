cla, clc, clear
hold all
axis([-200,200,-200,200,-200,200])
xlabel('X axis, mm')
ylabel('Y axis, mm')
zlabel('Z axis, mm')

% Link parameters
a2 = 100;   % mm
d4 = 120;   % mm

pp = [0,0,0,1]';

px = [1,0,0,0]';
py = [0,1,0,0]';
pz = [0,0,1,0]';

p4 = [0,0,0,1]';
p3 = [0,0,0,1]';
p2 = [0,0,0,1]';
p1 = [0,0,0,1]';
p0 = [0,0,0,1]';

theta = [0,0,0,0]';

% Transformation matricies
A1 = [  cos(theta(1)),  0,  sin(theta(1)),  0;
        sin(theta(1)),  0,  -cos(theta(1)), 0;
        0,              1,  0,              0;
        0               0,  0,              1];
    
A2 = [  cos(theta(2)),    -sin(theta(2)),       0,  a2*cos(theta(2));
        sin(theta(2)),     cos(theta(2)),       0,  a2*sin(theta(2));
        0,                  0,                  1,  0;
        0,                  0,                  0,  1];

A3 = [  cos(theta(3)),  0,  -sin(theta(3)),  0;
        sin(theta(3)),  0,  cos(theta(3)), 0;
        0,              -1,  0,              0;
        0               0,  0,              1];
    
A4 = [  cos(theta(4)),    -sin(theta(4)),       0,  0;
        sin(theta(4)),     cos(theta(4)),       0,  0;
        0,                  0,                  1,  d4;
        0,                  0,                  0,  1];

% Frame scaler
f_sc = 10;
% Line width
l_wi = 4;
    
dt = 0.1;    
for t = 0:dt:60
    cla
    
%    theta = theta + [dt,dt,dt,dt]';
    th_x = 0;
    th_y = t;
    th_z = 0;
    
    Rx = [  1,  0,  0;
            0,  cos(th_x),  -sin(th_x);
            0,  sin(th_x),  cos(th_x)];
        
    Ry = [  cos(th_y),  0,  sin(th_y);
            0,          1,  0;
            -sin(th_y), 0,  cos(th_y)];
    
    Rz = [  cos(th_z),  -sin(th_z), 0;
            sin(th_z),  cos(th_z),  0;
            0,          0,          1];
        
    R_eff = Rx*Ry*Rz;
%     R_eff = [  0.9603,  -0.1967, -0.1977;
%                0.1967,  0.9803,  -0.0198;
%                0.1977,  -0.0198,  0.9801];
        
    %p_eff = [75.2821,7.5534,127.5913]';
    p_eff = [100*cos(2*t),100*cos(t),100*sin(t)]';
    x_feff = f_sc*R_eff(:,1) + p_eff;
    y_feff = f_sc*R_eff(:,2) + p_eff;
    z_feff = f_sc*R_eff(:,3) + p_eff;

    %theta = nao_arm_ik(R_eff,p_eff,a2,d4)
    theta = nao_arm_ik_p(p_eff,a2,d4);
    
    % Plot target frame
    line([p_eff(1),x_feff(1)],[p_eff(2),x_feff(2)],[p_eff(3),x_feff(3)], 'Color', 'r', 'LineWidth', l_wi);
    line([p_eff(1),y_feff(1)],[p_eff(2),y_feff(2)],[p_eff(3),y_feff(3)], 'Color', 'g', 'LineWidth', l_wi);
    line([p_eff(1),z_feff(1)],[p_eff(2),z_feff(2)],[p_eff(3),z_feff(3)], 'Color', 'b', 'LineWidth', l_wi);

    %theta = [0.1,0.1,0.1,0.1]';

    % Transformation matricies
    A1 = [  cos(theta(1)),  0,  sin(theta(1)),  0;
            sin(theta(1)),  0,  -cos(theta(1)), 0;
            0,              1,  0,              0;
            0               0,  0,              1];
    
    A2 = [  cos(theta(2)),    -sin(theta(2)),       0,  a2*cos(theta(2));
            sin(theta(2)),     cos(theta(2)),       0,  a2*sin(theta(2));
            0,                  0,                  1,  0;
            0,                  0,                  0,  1];

    A3 = [  cos(theta(3)),  0,  -sin(theta(3)),  0;
            sin(theta(3)),  0,  cos(theta(3)), 0;
            0,              -1,  0,              0;
            0               0,  0,              1];
    
    A4 = [  cos(theta(4)),    -sin(theta(4)),       0,  0;
            sin(theta(4)),     cos(theta(4)),       0,  0;
            0,                  0,                  1,  d4;
            0,                  0,                  0,  1];
    
    %A1*A2*A3*A4
        
    p0 = pp;
    p1 = A1*pp;
    p2 = A1*A2*pp;
    p3 = A1*A2*A3*pp;
    p4 = A1*A2*A3*A4*pp;
    
    x0 = f_sc*px + p0;
    y0 = f_sc*py + p0;
    z0 = f_sc*pz + p0;
    
    x1 = f_sc*A1*px + p1;
    y1 = f_sc*A1*py + p1;
    z1 = f_sc*A1*pz + p1;
    
    x2 = f_sc*A1*A2*px + p2;
    y2 = f_sc*A1*A2*py + p2;
    z2 = f_sc*A1*A2*pz + p2;
    
    x3 = f_sc*A1*A2*A3*px + p3;
    y3 = f_sc*A1*A2*A3*py + p3;
    z3 = f_sc*A1*A2*A3*pz + p3;
    
    x4 = f_sc*A1*A2*A3*A4*px + p4;
    y4 = f_sc*A1*A2*A3*A4*py + p4;
    z4 = f_sc*A1*A2*A3*A4*pz + p4;
    
    % Plot frames
    % Frame zero
    line([p0(1),x0(1)],[p0(2),x0(2)],[p0(3),x0(3)], 'Color', 'r', 'LineWidth', l_wi);
    line([p0(1),y0(1)],[p0(2),y0(2)],[p0(3),y0(3)], 'Color', 'g', 'LineWidth', l_wi);
    line([p0(1),z0(1)],[p0(2),z0(2)],[p0(3),z0(3)], 'Color', 'b', 'LineWidth', l_wi);
    % Frame one
    line([p1(1),x1(1)],[p1(2),x1(2)],[p1(3),x1(3)], 'Color', 'r', 'LineWidth', l_wi);
    line([p1(1),y1(1)],[p1(2),y1(2)],[p1(3),y1(3)], 'Color', 'g', 'LineWidth', l_wi);
    line([p1(1),z1(1)],[p1(2),z1(2)],[p1(3),z1(3)], 'Color', 'b', 'LineWidth', l_wi);
    % Frame two
    line([p2(1),x2(1)],[p2(2),x2(2)],[p2(3),x2(3)], 'Color', 'r', 'LineWidth', l_wi);
    line([p2(1),y2(1)],[p2(2),y2(2)],[p2(3),y2(3)], 'Color', 'g', 'LineWidth', l_wi);
    line([p2(1),z2(1)],[p2(2),z2(2)],[p2(3),z2(3)], 'Color', 'b', 'LineWidth', l_wi);
    % Frame three
    line([p3(1),x3(1)],[p3(2),x3(2)],[p3(3),x3(3)], 'Color', 'r', 'LineWidth', l_wi);
    line([p3(1),y3(1)],[p3(2),y3(2)],[p3(3),y3(3)], 'Color', 'g', 'LineWidth', l_wi);
    line([p3(1),z3(1)],[p3(2),z3(2)],[p3(3),z3(3)], 'Color', 'b', 'LineWidth', l_wi);
    % Frame four
    line([p4(1),x4(1)],[p4(2),x4(2)],[p4(3),x4(3)], 'Color', 'r', 'LineWidth', l_wi);
    line([p4(1),y4(1)],[p4(2),y4(2)],[p4(3),y4(3)], 'Color', 'g', 'LineWidth', l_wi);
    line([p4(1),z4(1)],[p4(2),z4(2)],[p4(3),z4(3)], 'Color', 'b', 'LineWidth', l_wi);
    
    % Plot links
    line([p0(1),p1(1)],[p0(2),p1(2)],[p0(3),p1(3)], 'Color', 'k', 'LineWidth', l_wi);
    line([p1(1),p2(1)],[p1(2),p2(2)],[p1(3),p2(3)], 'Color', 'k', 'LineWidth', l_wi);
    line([p2(1),p3(1)],[p2(2),p3(2)],[p2(3),p3(3)], 'Color', 'k', 'LineWidth', l_wi);
    line([p3(1),p4(1)],[p3(2),p4(2)],[p3(3),p4(3)], 'Color', 'k', 'LineWidth', l_wi);
    
    pause(dt);
end