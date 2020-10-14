function [dX] = dynamics(~,X)
    dX = [X(4:6); zeros(3,1)];
end