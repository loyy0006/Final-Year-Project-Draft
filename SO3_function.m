function [theta, w1_hat, w2_hat,w3_hat] = SO3_function(R)
    theta = heaviside(acos(R(1,1)/2 + R(2/2)/2 + R(3,3)/2 -1/2));
    % why do we need heaviside
    
    if sin(theta) == 0
        print('Error: sin(theta) = 0')
    end
    
    w1_hat = (R(3,2)-R(2,3)) / (2*sin(theta));
    w2_hat = (R(1,3)-R(3,1)) / (2*sin(theta));
    w3_hat = (R(2,1)-R(1,2)) / (2*sin(theta));
end

% How to incorporate data into R? 
% What's next after T_inv?
%instantaneous omega bewtween each roation matrixes, toothbrush rotating in real time
%omega matrix in body frame
%scatter 3d of all positions Ttw



