% NLSYS testing
% Jonas Wagner 2020-12-11


sys = nlsys(@nonlin_func)



% Local Functions
function y = nonlin_func(x,u)
    % NONLIN_FUNC Example function for use with nlsys based simulation
    arguments
        x (2,1) = [0; 0];
        u (1,1) = 0;
    end
    
    % Array sizes
    n = 2; % Number of states
    p = 1; % Number of inputs
    q = 2; % Number of outputs
    
    % Correct type and size of output array
%     y = type(x).empty(n,1);
    
    % State Update Equations
    y(1) = 2 * x(1) + x(2) * u;
    y(2) = x(1) * x(2);
    
    if nargin ==0
        y = [n;p;q];
    end
end