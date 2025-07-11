function drawSteeringCar(X, l, height, width)
hold on
x = X(1); y = X(2); % positions
theta = X(3) - pi/2; % orientation
phi = X(4); % steering angle
h = height; w = width;

Wframe = [x; y] + rot2D(theta)*[w, -w, -w*1.5, w*1.5;
                                h, h,  -h, -h];

drawRec(Wframe(1,:),Wframe(2,:), 'facecolor', [255, 64, 105]/255, 'facealpha', 0.8)




hw = l/3; ww = hw/2;
pw = 0.675*w; ph = 0.5*h;

% axes
C1 = [x; y] + rot2D(theta)*[pw; ph];
C2 = [x; y] + rot2D(theta)*[-pw; ph];
C3 = [x; y] + rot2D(theta)*[-pw; -ph];
C4 = [x; y] + rot2D(theta)*[pw; -ph];
plot([C1(1),C2(1)], [C1(2),C2(2)], 'k','linewidth',3)
plot([C3(1),C4(1)], [C3(2),C4(2)], 'k','linewidth',3)

Cc1 = (C1+C2)/2; Cc2 = (C3+C4)/2; 
plot([Cc1(1),Cc2(1)], [Cc1(2),Cc2(2)], 'k','linewidth',3)

% wheels
W1 = [x; y] + rot2D(theta)*([pw; ph] + rot2D(phi)*[ww, ww, -ww, -ww;
    hw, -hw, -hw, hw]);
W2 = [x; y] + rot2D(theta)*([-pw; ph] + rot2D(phi)*[ww, ww, -ww, -ww;
    hw, -hw, -hw, hw]);
W3 = [x; y] + rot2D(theta)*([-pw; -ph] + [ww, ww, -ww, -ww;
    hw, -hw, -hw, hw]);
W4 = [x; y] + rot2D(theta)*([pw; -ph] + [ww, ww, -ww, -ww;
    hw, -hw, -hw, hw]);

drawRec(W1(1,:),W1(2,:), 'facecolor', [48, 196, 255]/255) %front wheels
drawRec(W2(1,:),W2(2,:), 'facecolor', [48, 196, 255]/255)  %front wheels
drawRec(W3(1,:),W3(2,:))
drawRec(W4(1,:),W4(2,:))







axis equal
grid on
end

function drawRec(X, Y, varargin)
if nargin==2
    patch('XData',X,'YData',Y,'FaceAlpha', 1);
else
    patch('XData',X,'YData',Y, varargin{:});
end
end
function R = rot2D(angle)
R = [cos(angle), -sin(angle);
    sin(angle), cos(angle)];
end