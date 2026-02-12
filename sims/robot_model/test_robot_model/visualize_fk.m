%% ChatGPT Generated 5.1 Visualization

function visualize_fk(DH, q)

[~, ~, ~, T_all] = fk(DH, q);

figure; hold on; grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title("FK Link Visualization");
view(3);

num_links = size(DH,1);
points = zeros(3, num_links+1);
points(:,1) = [0;0;0];

for i = 1:num_links
    points(:, i+1) = T_all(1:3,4,i);
end

plot3(points(1,:), points(2,:), points(3,:), 'ro-', ...
    'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','r');

end
