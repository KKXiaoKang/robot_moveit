function Cov_dq_estimated = get_Cov_dq_estimated(obj)
Cov_dq_estimated = diag([obj.Cov_Euler_rates(1,1); obj.Cov_Euler_rates(2,2); obj.Cov_Euler_rates(3,3);...
                        obj.CovVel_abduction; obj.CovVel_rotation; obj.CovVel_thigh; obj.CovVel_knee; obj.CovVel_qj1; obj.CovVel_qj2; obj.CovVel_toe; ... 
                        obj.CovVel_abduction; obj.CovVel_rotation; obj.CovVel_thigh; obj.CovVel_knee; obj.CovVel_qj1; obj.CovVel_qj2; obj.CovVel_toe]);
end

