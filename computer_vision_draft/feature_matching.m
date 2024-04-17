imds = imageDatastore("computer_vision_draft\capture2\")

for i = 1:length(imds.Files)
    image1 = imds.readimage(i);
    image1 = im2gray(image1);
    image2 = imds.readimage(i+1);
    image2 = im2gray(image2);

    %Measure time to match features
    tic 
    points1 = detectSIFTFeatures(image1);
    points2 = detectSIFTFeatures(image2);
    % points1 = detectORBFeatures(image1);
    % points2 = detectORBFeatures(image2);
    
    % points1 = points1.selectStrongest(100);
    % points2 = points2.selectStrongest(100);
    
    
    [features1, validPoints1] = extractFeatures(image1, points1,"Method","SIFT");
    [features2, validPoints2] = extractFeatures(image2, points2,"Method","SIFT");
    
    indexPairs = matchFeatures(features1,features2,"Method","Approximate","Unique",true);
    t = toc;
    
    matchingFrequency = 1/t;
    fprintf("The matching frequency is : %.2f Hz \n",matchingFrequency)
    fprintf("Total number of matches is : %i \n",length(indexPairs))
    
    
    %Plot
    matchedPoints1 = validPoints1(indexPairs(:,1),:);
    matchedPoints2 = validPoints2(indexPairs(:,2),:);
    
    figure(1)
    imshow(image1);
    hold on;
    plot(points1.selectStrongest(100))
    hold off
    
    figure(2)
    imshow(image2);
    hold on;
    plot(points2.selectStrongest(100))
    hold off
    
    figure(3) 
    showMatchedFeatures(image1,image2,matchedPoints1,matchedPoints2,"montage");

end