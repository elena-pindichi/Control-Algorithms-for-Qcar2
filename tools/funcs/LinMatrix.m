function M = LinMatrix(eta, Delta, L)
s1 = sin(eta(1)+eta(2)); c1 = cos(eta(1)+eta(2)); 
M = [cos(eta(1))-tan(eta(2))*(sin(eta(1))+Delta*s1/L), -Delta*s1;
     sin(eta(1))+tan(eta(2))*(cos(eta(1))+Delta*c1/L),  Delta*c1];
end