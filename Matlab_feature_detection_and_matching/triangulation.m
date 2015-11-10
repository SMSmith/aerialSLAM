function [motion] = triangulation(matchFeat)
% Get the difference in location and angle between the two frames
distances_0 = matchFeat.frame0.distances;
distances_1 = matchFeat.frame1.distances;
matchMetric = matchFeat.matchMetric;

% Save triangulation motion
motion.dx = 0;
motion.dy = 0;
motion.dz = 0;
motion.dr = 0;
motion.dp = 0;
motion.dy = 0;
end