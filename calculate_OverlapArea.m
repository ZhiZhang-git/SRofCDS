% % Define two overlapping polygons
% poly1_x = [0 2 2 0];  % x-coordinates of Polygon 1
% poly1_y = [0 0 2 2];  % y-coordinates of Polygon 1
% poly2_x = [1 3 3 1];  % x-coordinates of Polygon 2
% poly2_y = [1 1 3 3];  % y-coordinates of Polygon 2
% poly3_x = [1 3 3 1];  % x-coordinates of Polygon 2
% poly3_y = [1 1 3 3];  % y-coordinates of Polygon 2
% 
% 
% 
% 
% 
% % Create polyshape objects
% poly1 = polyshape(poly1_x, poly1_y);
% poly2 = polyshape(poly2_x, poly2_y);
% poly3 = polyshape(poly3_x, poly3_y);
% 
% 
% % Union of the polygons
% union_poly = union(poly1, poly2);
% union_poly=union(union_poly,poly3);
% 
% % % Plot the polygons and their union
% % figure;
% % plot(poly1, 'FaceColor', 'blue', 'FaceAlpha', 0.5);
% % hold on;
% % plot(poly2, 'FaceColor', 'red', 'FaceAlpha', 0.5);
% % hold on
% % plot(poly3, 'FaceColor', 'red', 'FaceAlpha', 0.5);
% % plot(union_poly, 'FaceColor', 'green', 'FaceAlpha', 0.5);
% % legend('Polygon 1', 'Polygon 2', 'Union');
% % hold off;
% 
% % Compute the area of the union
% area_union = area(union_poly);
% disp(['Area of the union: ', num2str(area_union)]);


% Parameters
numRegions = 100;

% Generate random polygons
polygons = [];
for i = 1:numRegions
    % Random vertices for each polygon
    x = rand(1, 4) * 10; % x-coordinates
    y = rand(1, 4) * 10; % y-coordinates
    polygons = [polygons, polyshape(x, y)]; %#ok<AGROW>
end

% Compute the union of all polygons
combinedPolygon = polygons(1);
for i = 2:numRegions
    combinedPolygon = union(combinedPolygon, polygons(i));
end

% Calculate the area of the combined region
areaUnion = area(combinedPolygon);

% Display results
disp(['Total area of 100 overlapping polygons: ', num2str(areaUnion)]);

% Visualization
figure;
plot(combinedPolygon);
title('Combined Polygon (Union of All Regions)');
