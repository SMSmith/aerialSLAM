function [motion] = triangulation(matchFeat)
% Get the difference in location and angle between the two frames
motion = matchFeat;
% Save triangulation motion
motion.dx = 0;
motion.dy = 0;
motion.dz = 0;
motion.dr = 0;
motion.dp = 0;
motion.dy = 0;
end