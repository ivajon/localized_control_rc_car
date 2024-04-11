%Based on the algorithm developed in
%https://www.researchgate.net/publication/257694570_Real-time_monocular_image-based_path_detection

imds = imageDatastore("capture1\"); %Get images

for i = 1:length(imds.Files)   
    %Step 1 - crop image at horizon, not necessary, could be done "by hand" if
    %we don't need to see above the floor.
    image1 =  readimage(imds,i);

    % image1 = imcrop(image1,[0 300 600 500]);
    figure(1)
    idisp(image1)
    
    %Step 2 - blur and color space conversion
    for i = 1:3
        image1(:,:,i) = medfilt2(image1(:,:,i)); %3x3 median filter
    end
    image1_hsv = rgb2hsv(image1);
    
    %Step 3 - superpixel segmentation
    N = 100;
    image_gray = im2gray(image1);
    [L,numLabels] = superpixels(image_gray,N);
    
    %Display superpixel image
    outputImage = zeros(size(image1),'like',image1);
    idx = label2idx(L);
    numRows = size(image1,1);
    numCols = size(image1,2);
    for labelVal = 1:numLabels
        redIdx = idx{labelVal};
        greenIdx = idx{labelVal}+numRows*numCols;
        blueIdx = idx{labelVal}+2*numRows*numCols;
        outputImage(redIdx) = mean(image1(redIdx));
        outputImage(greenIdx) = mean(image1(greenIdx));
        outputImage(blueIdx) = mean(image1(blueIdx));
    end    
    
    figure(2)
    imshow(outputImage)
end
