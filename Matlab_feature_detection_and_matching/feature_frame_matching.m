% Between Frame Matching
for i = 0:634
    if i < 10
        num = sprintf('00%i',i);
    elseif i < 100
        num = sprintf('0%i',i);
    else
        num = sprintf('%i',i);
    end
    f1 = sprintf('left%s.jpg',num);
    f2 = sprintf('right%s.jpg',num);
    I1 = rgb2gray(imread(f1));
    I2 = rgb2gray(imread(f2));
    points1 = detectSURFFeatures(I1,'MetricThreshold',2000);
    points2 = detectSURFFeatures(I2,'MetricThreshold',2000);
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
end