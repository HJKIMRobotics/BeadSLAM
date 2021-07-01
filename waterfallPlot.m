% This script generates the "waterfall plot" from beadSLAM
% beadSLAM needs to be run first

vIdx = [1:size(sEstStore,2)];

figure(7)
title('Vehicle position history')
clf
hold on
vehicleX = sEstStore(1,:);
vehicleP = sqrt(PssEstStore(1, :));
plot(vehicleX-2*vehicleP, vIdx, '--', 'LineWidth', 2, 'Color', [0 0 0]);
plot(vehicleX, vIdx, 'Color', [0 0 0]);
plot(vehicleX+2*vehicleP, vIdx, '--', 'LineWidth', 2, 'Color', [0 0 0]);
plot(sTrueStore(1, :), vIdx, 'LineWidth', 2, 'Color', [0 0 0]);

figure(8)
title('Landmark history')
clf
hold on

colours = distinguishable_colors(numberOfLandmarks);

for m = 1 : numberOfLandmarks
    landmarkX = sEstStore(2 + m,:);
    landmarkP = sqrt(PssEstStore(2 + m, :));
    plot(landmarkX-2*landmarkP, vIdx, '--', 'LineWidth', 2, 'Color', colours(m, :));
    plot(landmarkX, vIdx, 'Color', colours(m, :));
    plot(landmarkX+2*landmarkP, vIdx, '--', 'LineWidth', 2, 'Color', colours(m, :));
    plot(sTrueStore(2+m, :), vIdx, 'LineWidth', 2, 'Color', colours(m, :));
end