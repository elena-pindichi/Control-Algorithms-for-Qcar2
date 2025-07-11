function z = LinOutput(q,Delta,L)
z = [q(1)+L*cos(q(3))+Delta*cos(q(3)+q(4));
     q(2)+L*sin(q(3))+Delta*sin(q(3)+q(4))];
end