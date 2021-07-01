% This script illustrates the effects of cross correlation on two random
% variables x and y. Both are zero mean.

% Pick the correlation coefficient. This is in the range [-1,1].
sigmaXY = -0.7;

% The covariances
sigmaXX = 4;
sigmaYY = 4;

% Number of samples
N = 1000;

% Width of the figure samples
n = 100;

% Build the covariance matrix
P = [sigmaXX sigmaXY*sqrt(sigmaXX*sigmaYY);sigmaXY*sqrt(sigmaXX*sigmaYY) sigmaYY];

% Create the matrix square root. The 1e-12 is added to ensure numerical
% stability
PSqrtm = sqrtm(P + eps * eye(2));

% Standard deviation in Y - used for plotting
stdY = max(sqrt(sigmaXX), sqrt(sigmaYY));

% Create the figure
figure(1)
clf
hold on
axis([0 n -5*stdY 5*stdY]);
axis square

% Sample
Z = PSqrtm * randn(2, N);

% Plotting. Use handles to speed stuff up

% This version does a "tickertape" display
if (false)
    H(1) = plot(NaN, NaN, 'r', 'LineWidth', 2);
    H(2) = plot(NaN, NaN, 'k');
    H(3) = plot(NaN, NaN, 'b');
    for startIdx = 1 : N - n 
        endIdx = startIdx + n - 1;
        set(H(1), 'XData', 1:n, 'YData', Z(1, startIdx : endIdx));
        set(H(2), 'XData', 1:n, 'YData', Z(2, startIdx : endIdx));
        set(H(3), 'XData', 1:n, 'YData', Z(1, startIdx : endIdx)-Z(2, startIdx : endIdx));
        drawnow
        pause(0.02)
    end
end

% This version has "bouncing objects
if (true)
    H(1) = plot(NaN, NaN, 'r*', 'MarkerSize', 5);
    H(2) = plot(NaN, NaN, 'g*', 'MarkerSize', 5);
    H(3) = plot(NaN, NaN, 'r', 'LineWidth', 2);
    H(4) = plot(NaN, NaN, 'g', 'Linewidth', 2);
    H(5) = plot(NaN, NaN, 'k');

    for idx = 1 : N
        set(H(1), 'XData', n/4, 'YData', Z(1, idx));
        set(H(2), 'XData', 3*n/4, 'YData', Z(2, idx));
        trail = max(idx-10,1):idx;
        set(H(3), 'XData', n/4*ones(length(trail), 1), 'YData', Z(1, trail));
        set(H(4), 'XData', 3*n/4*ones(length(trail), 1), 'YData', Z(2, trail));
        set(H(5), 'XData', [n/4 3*n/4], 'YData', Z(1:2, idx)');
        drawnow
        pause(0.02)
    end
end

% This draws a 2D plot
if (false)
    H(1) = plot(NaN, NaN, 'r*', 'MarkerSize', 5);
    H(2) = plot(NaN, NaN, 'r', 'LineWidth', 2);

    for idx = 1 : N
        set(H(1), 'XData', Z(1, idx), 'YData', Z(2, idx));
        trail = max(idx-10,1):idx;
        set(H(2), 'XData', Z(1, trail), 'YData', Z(2, trail));
        drawnow
        pause(0.02)
    end
end



