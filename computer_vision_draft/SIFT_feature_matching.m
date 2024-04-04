%Simple script for determining feasability of using SIFT features
%Note that the distance between the images is several meters,
%A worst case scenario

image1 = imread("sample_images\20240404_111521.jpg");
image2 = imread("sample_images\\20240404_111523.jpg");
image1 = im2gray(image1);
image1 = imresize(image1,[600 800]);

image2 = im2gray(image2);
image2 = imresize(image2,[600 800]);

%Measure time to match features
tic 
points1 = detectSIFTFeatures(image1);
points2 = detectSIFTFeatures(image2);
points1 = points1.selectStrongest(100);
points2 = points2.selectStrongest(100);


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