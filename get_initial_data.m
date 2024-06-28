clearvars
close all

load('data_from_simulator.mat')
rng(42)

ns = size(ego_vels, 2);
n_samples = 5000;
random_per = randperm(ns, n_samples);

% initialise output 
headways_out = zeros(n_samples, 1);
ego_vels_out = zeros(n_samples, 1);
lead_vel_out = zeros(n_samples, 1);
max_vels_out = zeros(n_samples, 1);
opt_vels_out_dsm = zeros(n_samples, 600);
opt_vels_out_trade = zeros(n_samples, 600);
opt_vels_out_min = zeros(n_samples, 600);

tStart = tic;
counter = 0;

for i=1:n_samples
    idx = random_per(i);
    
    x_l_0 = headways(idx); 
    v_0 = ego_vels(idx);
    v_l_0 = lead_vels(idx); 
    v_max = max_vels(idx);

    t_lead = 0:0.1:60;
    v_lead = v_l_0 * ones(size(t_lead));

    x_e_0 = 0;

    s_d = 2.5;
    T_d = 1.5;

    SoC_0 = 0.7;
    
    asdkkasd
    
    for W1 = [1, 4000, 100000]
        
        new_vel = EV_SimpleDMMain_func(W1, t_lead, ...
                v_lead, v_0, x_e_0, x_l_0, v_max, s_d, T_d, SoC_0);
        
        divider = int32(size(new_vel, 1)/2);
        
        time = new_vel(1:divider);
        v = new_vel(divider+1:end);

        v_interp = interp1(time, v, t_lead, 'linear');
        v_append = v_interp(2:end);
    
        if W1 == 1
            opt_vels_out_dsm(i, :) = v_append;
        elseif W1 == 4000
            opt_vels_out_trade(i, :) = v_append;            
        else
            opt_vels_out_min(i, :) = v_append;
        end
        
        counter = counter + 1;
        time  = toc(tStart);
        
    end
    
    % append the newly calculated values
    headways_out(i) = x_l_0;
    ego_vels_out(i) = v_0;
    lead_vel_out(i) = v_l_0;
    max_vels_out(i) = v_max;

    % save the solutions
    save('sol_space_dsm', 'headways_out', 'ego_vels_out', 'lead_vel_out', ...
        'max_vels_out', 'opt_vels_out_dsm')
    
    save('sol_space_trade', 'headways_out', 'ego_vels_out', 'lead_vel_out', ...
        'max_vels_out', 'opt_vels_out_trade')

    save('sol_space_min', 'headways_out', 'ego_vels_out', 'lead_vel_out', ...
        'max_vels_out', 'opt_vels_out_min')
    
end