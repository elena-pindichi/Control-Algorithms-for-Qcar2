function xd = SteeringCar(x,u,L)
% L(m) is the constant parameter of the car
xd = [u(1)*cos(x(3));
    u(1)*sin(x(3));
    u(1)*tan(x(4))/L;
    u(2)];
end

