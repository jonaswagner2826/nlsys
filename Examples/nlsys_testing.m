% NLSYS testing
% Jonas Wagner 2020-12-11


sys = nlsys(@nonlin_state_func)

x_0 = [1,2];

sys2 = nlsys(@nonlin_state_func,@nonlin_output_func,x_0)


u_test = 1;
t_test = 1;
sys2.dx(u_test)
sys3 = sys2.update(u_test,t_test)
x = sys3.x

% Local Functions
function y = nonlin_state_func(x,u)
    % NONLIN_FUNC Example function for use with nlsys based simulation
    arguments
        x (2,1) = [0; 0];
        u (1,1) = 0;
    end
    
    % Array sizes
    n = 2; % Number of states
    p = 1; % Number of inputs

    % State Update Equations
    y(1,1) = 2 * x(1) + x(2) * u;
    y(2,1) = x(1) * x(2);
    
    if nargin ==0
        y = [n;p];
    end
end

function y = nonlin_output_func(x,u)
    % NONLIN_OUTPUT_FUNC Example function for use with nlsys
    arguments
        x (2,1) = [0; 0];
        u (1,1) = 0;
    end
    
    % Array sizes
    n = 2; % Number of states
    p = 1; % Number of inputs
    q = 2; % Number of outputs
    
    % State Update Equations
    y(1,1) = x(1) + u;
    y(2,1) = x(2) * u;
    
    if nargin ==0
        y = [n;p;q];
    end
end