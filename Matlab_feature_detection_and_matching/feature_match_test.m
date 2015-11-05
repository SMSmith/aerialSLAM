% Between Frame Matching
for i = 0:633
    if i < 9
        num1 = sprintf('00%i',i);
        num2 = sprintf('00%i',i+1);
    elseif i == 9
        num1 = sprintf('00%i',i);
        num2 = sprintf('0%i',i+1);
    elseif i < 99
        num1 = sprintf('0%i',i);
        num2 = sprintf('0%i',i+1);
    elseif i == 99
        num1 = sprintf('0%i',i);
        num2 = sprintf('%i',i+1);
    else
        num1 = sprintf('%i',i);
        num2 = sprintf('%i',i+1);
    end
    f1 = sprintf('left%s.jpg',num1);
    f2 = sprintf('left%s.jpg',num2);
    I1 = rgb2gray(imread(f1));
    I2 = rgb2gray(imread(f2));
    points1 = detectSURFFeatures(I1);
    points2 = detectSURFFeatures(I2);
    [f1, vpts1] = extractFeatures(I1, points1);
    [f2, vpts2] = extractFeatures(I2, points2);
    [indexPairs, matchmetric] = matchFeatures(f1, f2) ;
    matchedPoints1 = vpts1(indexPairs(:, 1), :);
    matchedPoints2 = vpts2(indexPairs(:, 2), :);
    if(i>0)
        clf('reset')
    end
    figure(1); ax = axes;
    showMatchedFeatures(I1,I2,matchedPoints1,matchedPoints2,'Parent',ax);
%     title(sprintf('left frame %i',i));
%     getframe(gcf);
end