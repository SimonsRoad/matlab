function P = get_relative_pose(initial_pose, new_pose)
  R1 = initial_pose(1:3,1:3);
  t1 = initial_pose(1:3,4);
  R2 = new_pose(1:3,1:3);
  t2 = new_pose(1:3,4);
  R_relative = inv(R2)*R1
  t_relative = inv(R2) * (t1 - t2)
  P(1:3,1:3) = R_relative;
  P(1:3, 4) = t_relative;
  P(4, 1:4) = [ 0; 0; 0; 1 ];
end

