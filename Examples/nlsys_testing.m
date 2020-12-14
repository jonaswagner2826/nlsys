% NLSYS testing
% Jonas Wagner 2020-12-11
clear
close all

% Basic nlsys constroctor with a nonlinear state equation
sys = nlsys(@nonlin_state_func)


% nlsys constructor with nonlinear state and output equations and initial
% condition
x_0 = [1,2];
sys2 = nlsys(@nonlin_state_func,@nonlin_output_func,x_0)

% nlsys update operations... I wonder if using handle class would be better
% so that (in this example) the reference sys2 would be directly updated
% with the update method, but this is how it is done in the control systems
% toolbox...
u_test = 1;
t_test = 1;
sys2.dx(u_test)
sys3 = sys2.update(u_test,t_test)
x_3 = sys3.x



% nlsys from ss object
A_test = eye(2);
B_test = [1; 0];
C_test = [1, 0];
D_test = 1;
sys_ss = ss(A_test,B_test,C_test,D_test)
sys4 = nlsys(sys_ss)

% nlsys from tf object
sys_tf = tf([1],[1 1 1])
sys5 = nlsys(sys_tf)



% testing interconnectivity functions
f1 = sys.f
f2 = sys5.f
nlsys.func_sum(f1,f2)
nlsys.func_sum(f1,f2,f1,f2)

f1 = @(x) x^2;
f2 = @(x) x^3;
f3 = nlsys.func_conv(f1,f2,f1)
syms x
f3(x)



% sys7 = nlsys.parrellel(sys2,sys3)

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