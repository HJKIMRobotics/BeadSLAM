function crossCorrelationExamples()

% Cross correlation examples

P = [1 0;0 1];

plotXCorrelation(1, P, [0 0 1]);

P = [4 0;0 1];

plotXCorrelation(2, P, [0 0 1]);

P = [4 0.5;0.5 1];

plotXCorrelation(3, P, [0 0 1]);

P = [4 1.999999;1.999999 1];

plotXCorrelation(4, P, [0 0 1]);


end


function plotXCorrelation(figNo, P, colour)

theta = (0 : 5 : 365) * pi / 180;

pts = sqrtm(P) * [cos(theta);sin(theta)];

figure(figNo)

plot(pts(1,:), pts(2,:), 'Color', colour, 'LineWidth', 2);

P=round(P);

text(2, 2, sprintf('%d %d\n%d %d',P(1,1), P(1,2), P(2,1),P(2,2)));

axis([-3 3 -3 3])
axis square

print(sprintf('~/Desktop/%02d.jpg', figNo), '-djpeg100');

end