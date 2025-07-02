function model = forward_motion_model_v2(varargin)
     import casadi.*

    %% system dimensions
    nx = 12;
    nu = 1;

    %% system parameters
    if nargin > 1 && varargin{2} % parametric model
        v_h = SX.sym('v_h');  % head-truck velocity [m/s]
        param = v_h;
    else
        v_h = 5;    % head-truck velocity [m/s]
        param = [];
    end
    l_h = 6;            % Head-truck length
    l_t = 10;           % Trailer length
    l_wheel = 1;        % Wheel length
    w_wheel = 0.6;      % Wheel width
    %% named symbolic variables
    % State - Position
    x_h = SX.sym('x_h');                   % X-axis position of head-truck [m]
    y_h = SX.sym('y_h');                   % Y-axis position of head-truck [m]
    theta_hf = SX.sym('theta_hf');         % Heading angle of head-truck [rad]
    x_t = SX.sym('x_t');                   % X-axis position of trailer [m]
    y_t = SX.sym('y_t');                   % Y-axis position of trailer [m]
    theta_tf = SX.sym('theta_tf');         % Heading angle of trailer [rad]

    % State - Error
    e_h = SX.sym('e_h');                   % head-truck position distance error
    e_theta_hp = SX.sym('e_theta_hp');     % head-truck and path error
    e_theta_ht1 = SX.sym('e_theta_ht1');   % head-truck and trailer error
    e_t = SX.sym('e_t');                   % trailer position distance error
    e_theta_tp = SX.sym('e_theta_tp');     % trailer and path error
    e_theta_ht2 = SX.sym('e_theta_ht2');   % trailer and trailer error

    % Input
    delta = SX.sym('delta');               % steering angle [rad]

    %% (unnamed) symbolic variables
    x = vertcat(x_h, y_h, theta_hf, x_t, y_t, theta_tf, e_h, e_theta_hp, e_theta_ht1, e_t, e_theta_tp, e_theta_ht2);
    xdot = SX.sym('xdot', nx, 1);
    u = delta;

    %% dynamics
    f_expl_expr = vertcat(v_h * cos(theta_hf), ...                                 
                          v_h * sin(theta_hf), ...
                          (v_h/l_h)*tan(delta), ...
                          v_h * cos(theta_hf - theta_tf) * cos(theta_tf), ...
                          v_h * cos(theta_hf - theta_tf) * sin(theta_tf), ...
                          (v_h / l_t) * sin(theta_hf - theta_tf), ...
                          v_h * sin(e_theta_hp), ...
                          (v_h / l_h) * tan(delta), ...
                          v_h * ( tan(delta) / l_h - sin(e_theta_ht1) / l_t), ...
                          v_h * cos(e_theta_ht2) * sin(e_theta_tp), ...
                          (v_h / l_t) * sin(e_theta_ht2), ...
                          v_h * ( tan(delta) / l_h - sin(e_theta_ht1) / l_t));
    
    f_impl_expr = f_expl_expr - xdot;

    % discrete dynamics
    if nargin > 0
        delta_t = varargin{1};
        disc_dyn_expr = x + delta_t * f_expl_expr; % explicit Euler
    else
        disc_dyn_expr = [];
    end

    %% populate structure
    model = AcadosModel();
    model.x = x;
    model.xdot = xdot;
    model.u = u;
    model.p = param;

    model.f_expl_expr = f_expl_expr;
    model.f_impl_expr = f_impl_expr;
    model.disc_dyn_expr = disc_dyn_expr;

    % Constraints
    model.con_h_expr = u;  % input constraints

    model.name = 'forward_motion_v2';
end