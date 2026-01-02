% visualize_rotation.m
% Visualize rotation R, axis h and effect on sample vectors

% Given matrix (your example)
M = [4 -4 -7; 8 1 4; -1 -8 4];
R = M/9;

% Compute axis (eigenvector for lambda=1) and normalize
h_raw = null(R - eye(3));
h = h_raw(:,1);
h = h / norm(h);

% Compute angle (two ways)
S = (R - R.')/2;
v_vex = [S(3,2); S(1,3); S(2,1)];      % vex(S)
sin_theta = dot(h, v_vex);            % projection -> sin(theta)
theta_from_v = asin(max(min(sin_theta,1),-1));    % clamp for numeric issues
theta_from_trace = acos((trace(R)-1)/2);          % alternative

fprintf('h (unit) = [%g, %g, %g]\\n', h(1), h(2), h(3));
fprintf('theta (from vex) = %g rad = %g deg\\n', theta_from_v, rad2deg(theta_from_v));
fprintf('theta (from trace) = %g rad = %g deg\\n', theta_from_trace, rad2deg(theta_from_trace));

% Sample vectors (one orthogonal to h, one arbitrary)
% Ensure v1 is orthogonal to h: pick any vector not parallel then remove component along h
u = [1;0;0];
if abs(dot(u,h)) > 0.9, u = [0;1;0]; end
v1 = u - (dot(u,h))*h;       % make orthogonal
v1 = v1 / norm(v1);
v2 = [0.2; 0.7; -0.1];       % arbitrary direction
v2 = v2 / norm(v2);         % normalize for plotting

% Rotated vectors
Rv1 = R * v1;
Rv2 = R * v2;

% Long arrows for visibility
L = 1.2;

figure('Color','w'); hold on; grid on; axis equal;
xlabel('x'); ylabel('y'); zlabel('z');
view(40,22);

% Plot origin
plot3(0,0,0,'k.');

% Plot axis h (both directions) as a line and arrows
quiver3(0,0,0, L*h(1), L*h(2), L*h(3), 'LineWidth',3, 'MaxHeadSize',0.5, 'Color',[0 0.6 0]);
quiver3(0,0,0, -L*h(1), -L*h(2), -L*h(3), 'LineWidth',3, 'MaxHeadSize',0.5, 'Color',[0 0.6 0]);
text(L*h(1), L*h(2), L*h(3), '  h (axis)', 'Color',[0 0.4 0]);

% Plot original vectors (blue)
quiver3(0,0,0, L*v1(1), L*v1(2), L*v1(3), 'b', 'LineWidth',2, 'MaxHeadSize',0.4);
text(L*v1(1), L*v1(2), L*v1(3), '  v1 (orig)', 'Color','b');
quiver3(0,0,0, L*v2(1), L*v2(2), L*v2(3), 'b', 'LineWidth',2, 'MaxHeadSize',0.4);
text(L*v2(1), L*v2(2), L*v2(3), '  v2 (orig)', 'Color','b');

% Plot rotated vectors (red)
quiver3(0,0,0, L*Rv1(1), L*Rv1(2), L*Rv1(3), 'r', 'LineWidth',2, 'MaxHeadSize',0.4);
text(L*Rv1(1), L*Rv1(2), L*Rv1(3), '  R*v1', 'Color','r');
quiver3(0,0,0, L*Rv2(1), L*Rv2(2), L*Rv2(3), 'r', 'LineWidth',2, 'MaxHeadSize',0.4);
text(L*Rv2(1), L*Rv2(2), L*Rv2(3), '  R*v2', 'Color','r');

% Draw a small legend box
legend({'origin','axis h (both directions)','v (orig)','R*v (rotated)'}, 'Location','best');

% OPTIONAL: animate rotation from 0 to theta (uncomment to run)
%{
nsteps = 60;
for k=1:nsteps
    t = k/nsteps * theta_from_trace;
    Rt = expm(skew(h)*t);   % requires skew function below
    Rv1t = Rt * v1;
    Rv2t = Rt * v2;
    cla;
    plot3(0,0,0,'k.'); hold on; grid on; axis equal;
    quiver3(0,0,0, L*h(1), L*h(2), L*h(3), 'g','LineWidth',3);
    quiver3(0,0,0, L*Rv1t(1), L*Rv1t(2), L*Rv1t(3),'b','LineWidth',2);
    quiver3(0,0,0, L*Rv2t(1), L*Rv2t(2), L*Rv2t(3),'b','LineWidth',2);
    drawnow;
end
%}

% small helper: skew-symmetric matrix
function S = skew(u)
    S = [  0   -u(3)  u(2);
          u(3)   0   -u(1);
         -u(2) u(1)    0 ];
end