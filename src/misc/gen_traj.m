function [traj] = gen_traj(tspan)
    dt = tspan(2)-tspan(1);
    attitude = e2a(60,0,0,true);
    position = zeros(3,1);
    vel = [1;0;0];
    traj = zeros(length(tspan),12);
    time_until_turn = round(10 + (50-10)*rand);
    step_count = 0;
    for ii = 1:length(tspan)
        % Propagate:
        if step_count == time_until_turn
            rotation_new = e2a(0,0,10 + (50-10)*randn, true);
            attitude = attitude*rotation_new;
            vel = rotation_new*vel;
            time_until_turn = round(10 + (50-10)*rand); 
            step_count = 0;
        end
        X = rk4(@dynamics,dt,[position; vel]);
        position = X(1:3);
        
        % Store result:
        step_count = step_count + 1;
        traj(ii,:) = [reshape(attitude,1,[]), position'];
    end
end