function results = run_chung_kia(agents, leader, graph, T_s, x_init, x0_init, time, dt)

N  = length(agents);
n  = agents{1}.n;
M  = length(time);


x_hist  = zeros(n, N, M);
x0_hist = zeros(n, M);
u_hist  = zeros(N, M);

% current state
x  = x_init;    % n×N  stati follower
xl = x0_init;   % n×1  stato leader

% 
x_tk    = zeros(n, N);  % xⁱ(tₖ)
x0_tk   = zeros(n, 1);  % x⁰(tₖ)
Gk      = cell(N, 1);   % Gramiano G^i_k = G^i(T_s)
Gk_inv  = cell(N, 1);   % (G^i_k)⁻¹
eATk    = cell(N, 1);   % e^{Aⁱ Tₛ}

% ODE Lyapunov
% Ẇʲ = Aʲ Wʲ + Wʲ Aʲᵀ + Bʲ Bʲᵀ,  Wʲ(tₖ) = 0

W = cell(N, 1);
for j = 1:N
    W{j} = zeros(n, n);
end

deadzone = 15 * dt;

prev_epoch = -1;


for s = 1:M
    t    = time(s);
    k_ep = floor(t / T_s + 1e-10);   
    t_k  = k_ep * T_s;
    t_k1 = (k_ep + 1) * T_s;
    t_el = t - t_k;                   
    t_rm = t_k1 - t;                  

    
    if k_ep ~= prev_epoch
        prev_epoch = k_ep;
        x_tk  = x;
        x0_tk = xl;

        for i = 1:N
            Ai = agents{i}.A;
            Bi = agents{i}.B;

            
            Gki       = compute_finite_gramian(Ai, Bi, T_s);
            Gk{i}     = Gki;
            Gk_inv{i} = Gki \ eye(n);    % (G^i_k)⁻¹

            
            eATk{i} = expm(Ai * T_s);

          
            W{i} = zeros(n, n);
        end
    end


    x_hist(:, :, s) = x;
    x0_hist(:, s)   = xl;

    % control law
    u = zeros(N, 1);

    t_rm_s = max(t_rm, 1e-9);    

    for i = 1:N
        Ai = agents{i}.A;
        Bi = agents{i}.B;

        
        eAiT_rm    = expm(Ai' * t_rm_s);

        
        eAiTk_xik  = eATk{i} * x_tk(:, i);


        I_i   = double(ismember(i, graph.N0_in));
        d_out = sum(graph.A_adj(i, :));
        denom = I_i + d_out;

        if denom == 0
            u(i) = 0;
            continue;
        end

        w_l = I_i  / denom;
        w_f = 1.0  / denom;


        % v_l = x⁰(tₖ) − F^{i0} − e^{Aⁱ Tₛ} xⁱ(tₖ)     [F^{i0}=0]
        v_l = zeros(n, 1);
        if I_i
            v_l = x0_tk - eAiTk_xik;
        end

       
        %   v_f = Σⱼ aᵢⱼ { [Gʲₖ (Gʲₖ(t))⁻¹ (xʲ(t) − e^{Aʲ tel} xʲ(tₖ))]
        %                  +  [e^{Aʲ Tₛ} xʲ(tₖ) − e^{Aⁱ Tₛ} xⁱ(tₖ)] }
        
        v_f = zeros(n, 1);

        for j = 1:N
            if graph.A_adj(i, j)   
                Aj = agents{j}.A;

                
                eAjTk_xjk = eATk{j} * x_tk(:, j);

                
                t2 = eAjTk_xjk - eAiTk_xik;

                
                t1 = zeros(n, 1);

                if t_el > deadzone
                    eAj_tel  = expm(Aj * t_el);
                    xj_free  = eAj_tel * x_tk(:, j);
                    xj_diff  = x(:, j) - xj_free;   

                    Phi_j = expm(Aj' * t_rm_s);
                    Gjkt  = W{j} * Phi_j;

                    
                    if rcond(Gjkt) > 1e-12
                        t1 = Gk{j} * (Gjkt \ xj_diff);
                    end
                end

                v_f = v_f + t1 + t2;
            end
        end

        % uⁱ = Bⁱᵀ e^{Aⁱᵀ(…)} (Gⁱₖ)⁻¹ (w_l v_l + w_f v_f)
        v    = w_l * v_l + w_f * v_f;
        u(i) = Bi' * eAiT_rm * (Gk_inv{i} * v);
    end

    u_hist(:, s) = u;

    
    for i = 1:N
        Ai = agents{i}.A;
        Bi = agents{i}.B;
        xi = x(:, i);
        ui = u(i);
        f  = @(z) Ai * z + Bi * ui;
        k1 = f(xi);
        k2 = f(xi + dt/2 * k1);
        k3 = f(xi + dt/2 * k2);
        k4 = f(xi + dt   * k3);
        x(:, i) = xi + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4);
    end

    
    f_l = @(z, s_t) leader_dynamics_ck(z, leader, s_t);
    k1 = f_l(xl,            t);
    k2 = f_l(xl + dt/2*k1,  t + dt/2);
    k3 = f_l(xl + dt/2*k2,  t + dt/2);
    k4 = f_l(xl + dt   *k3, t + dt);
    xl = xl + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4);

    
    % Ẇʲ = Aʲ Wʲ + Wʲ Aʲᵀ + Bʲ Bʲᵀ
    for j = 1:N
        Aj   = agents{j}.A;
        Bj   = agents{j}.B;
        dW   = Aj * W{j} + W{j} * Aj' + Bj * Bj';
        W{j} = W{j} + dt * dW;
    end
end

% output
results.x_hist     = x_hist;
results.x0_hist    = x0_hist;
results.u_hist     = u_hist;
results.time       = time;
results.pos        = reshape(x_hist(1, :, :), N, M);   % N×M
results.vel        = reshape(x_hist(2, :, :), N, M);   % N×M
results.leader_pos = x0_hist(1, :);                     % 1×M
results.leader_vel = x0_hist(2, :);                     % 1×M
end
