function R = end_effector_rotation(DH, q)
    [~, ~, R] = fk(DH, q);
end
