function obs_traj = update_obs_state(obs_num,obs_traj,N)
    obs_traj(1,:,:) = obs_traj(2,:,:);
    for i = 2 : N + 1
        for j = 1 : obs_num
            obs_traj(i,j,1) = obs_traj(i-1,j,1) +obs_traj(i-1,j,4) * cos(obs_traj(i-1,j,3)) ;
            obs_traj(i,j,2) = obs_traj(i-1,j,2) +obs_traj(i-1,j,4) * sin(obs_traj(i-1,j,3)) ;
            obs_traj(i,j,3) = obs_traj(i-1,j,3);
            obs_traj(i,j,4) = obs_traj(i-1,j,4);
        end
    end
end