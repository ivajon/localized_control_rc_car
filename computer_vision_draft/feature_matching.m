imds = imageDatastore("capture2\");
%I will 
% Methods = ["SIFT", "SIFT";
%            % "SURF", "SURF";
%            % "FAST", "FREAK";
%            "FAST", "SURF";
%             % "KAZE","KAZE";
%             "ORB","ORB"];

Methods = [
    "SURF", "SURF"; 
    % "FAST", "SURF";
    "ORB","ORB"
    ];

avgSpeed = zeros(length(Methods),1);
speed = zeros(length(Methods),length(imds.Files)-1);
avgMatches = zeros(length(Methods),1);
matches = zeros(length(Methods),length(imds.Files)-1);

for m = 1:length(Methods)
    for i = 1:length(imds.Files)-1
        image1 = imds.readimage(i);
        
        image2 = imds.readimage(i+1);
        
        
    
        %Measure time to match features
        tic 

        image1 = im2gray(image1);
        % image1 = imgaussfilt(image1,2);
        
        image2 = im2gray(image2);
        % image2 = imgaussfilt(image2,2);
        points1 = featureMethod(image1,Methods(m,1));
        points2 = featureMethod(image2,Methods(m,1));
        disp(length(points1))
    
        points1 = points1.selectStrongest(200);
        points2 = points2.selectStrongest(200);
        
        
        [features1, validPoints1] = extractFeatures(image1, points1,"Method",Methods(m,2));
        [features2, validPoints2] = extractFeatures(image2, points2,"Method",Methods(m,2));
        
        indexPairs = matchFeatures(features1,features2,"Method","Approximate","Unique",true);
        t = toc;
        avgSpeed(m) = avgSpeed(m)+t;
        speed(m,i) = t;
        avgMatches(m) = length(indexPairs)+avgMatches(m);
        matches(m,i) = length(indexPairs);
        matchingFrequency = 1/t;
        disp(Methods(m,:))
        fprintf("The matching frequency is : %.2f Hz \n",matchingFrequency)
        fprintf("Total number of matches is : %i \n",length(indexPairs))
        
        
        %Plot
        matchedPoints1 = validPoints1(indexPairs(:,1),:);
        matchedPoints2 = validPoints2(indexPairs(:,2),:);
        
        % figure(1)
        % imshow(image1);
        % hold on;
        % plot(points1.selectStrongest(100))
        % hold off
        % 
        % figure(2)
        % imshow(image2);
        % hold on;
        % plot(points2.selectStrongest(100))
        % hold off
        % 
        figure(3) 
        showMatchedFeatures(image1,image2,matchedPoints1,matchedPoints2);
    end
    
    avgSpeed(m) = avgSpeed(m)/(length(imds.Files)-1)
    avgFreq = 1./avgSpeed
    avgMatches(m) = avgMatches(m)/(length(imds.Files)-1)

    figure(4)
    histogram(matches(m,:))
    title("Match distribution")

    figure(5)
    histogram(1./speed(m,:))
    title("Frequency distribution")
end
    
   %  pause(5)
   % figure(4)
   %  histogram(matches(2,:),20)
   %  title("Match distribution")
   % 
   %  figure(5)
   %  histogram(1./speed(2,:),20)
   %  title("Frequency distribution")