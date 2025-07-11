function O = LinDyna(eta,L,Delta)
s1 = sin(eta(1)+eta(2)); c1 = cos(eta(1)+eta(2)); 
O = [sin(eta(2))*c1/L,             sin(eta(2))*s1/L;
    -sin(eta(2))*c1/L-s1/Delta,    -sin(eta(2))*s1/L-c1/Delta];
% G = [[1 0;
%      0 1];
%      O];
 
end

