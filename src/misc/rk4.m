function [X] = rk4(dynamics, dt, X, varargin)
    k1 = dt*dynamics(0,      X,          varargin{:});
    k2 = dt*dynamics(0.5*dt, X + 0.5*k1, varargin{:});
    k3 = dt*dynamics(0.5*dt, X + 0.5*k2, varargin{:});
    k4 = dt*dynamics(dt,     X + k3,     varargin{:});
    X = X + (1/6)*(k1 + 2*k2 + 2*k3 + k4);
end