%Based on the algorithm developed in
%https://www.researchgate.net/publication/257694570_Real-time_monocular_image-based_path_detection

%Which relies on
% Road classification
% https://www.roboticsproceedings.org/rss02/p05.pdf
%
% Fast segmentation algorithm(Quick Shift)
% https://www.robots.ox.ac.uk/~vedaldi/assets/pubs/vedaldi08quick.pdf

imds = imageDatastore("capture1\"); %Get images

for i = 1:length(imds.Files)   
    %Step 1 - crop image at horizon, not necessary, could be done "by hand" if
    %we don't need to see above the floor.
    I =  readimage(imds,i);

    I = imcrop(I,[0 201 640 280]);
    figure(1)
    idisp(I)
    
    %Step 2 - blur and color space conversion
    for i = 1:3
        I(:,:,i) = medfilt2(I(:,:,i)); %3x3 median filter
    end
    Ihsv = rgb2hsv(I); Might not be necessary to use both rgb and hsv

    
    %Step 3 - superpixel segmentation, or other faster segmentation method
    N = 100;
    I_gray = im2gray(I);
    [L,numLabels] = superpixels(I_gray,N);
    
    %Display superpixel image
    outputImage = zeros(size(I),'like',I);
    idx = label2idx(L);
    numRows = size(I,1);
    numCols = size(I,2);
    for labelVal = 1:numLabels
        redIdx = idx{labelVal};
        greenIdx = idx{labelVal}+numRows*numCols;
        blueIdx = idx{labelVal}+2*numRows*numCols;
        outputImage(redIdx) = mean(I(redIdx));
        outputImage(greenIdx) = mean(I(greenIdx));
        outputImage(blueIdx) = mean(I(blueIdx));
    end    
    
    figure(2)
    imshow(outputImage)


    %Step 4 - creating mixture of gaussians model
    % For each set of "training data", we segment into 3 groups using
    % k-means
    % Not implemented here, but it is possible to store an amount of
    % segments n>k from previous runs.
    % 
    % Each gaussian has three properties
    % Mean, covariance and mass
    numGauss = 3;

    floorArea = imcrop(I,[210 150 200 130]); %Assume floor infront of camera
    L_floor = imsegkmeans(floorArea,3);
    figure(3)
    imshow(labeloverlay(floorArea,L_floor,"Transparency",0.9))
end


