function p = end_effector_position(DH, q)
    [~, p, ~] = fk(DH, q);
end
