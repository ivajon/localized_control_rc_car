function featurePoints = featureMethod(image,method)
%featureMethod Detects features using selected method
    if method == "BRISK"
        featurePoints = detectBRISKFeatures(image);
    elseif method == "FAST"
        featurePoints = detectFASTFeatures(image);
    elseif method == "Harris"
        featurePoints = detectHarrisFeatures(image);
    elseif method == "KAZE"
        featurePoints = detectKAZEFeatures(image);
    elseif method == "MinEigen"
        featurePoints = detectMinEigenFeatures(image);
    elseif method == "MSER"
        featurePoints = detectMSERFeatures(image);
    elseif method == "ORB"
        featurePoints = detectORBFeatures(image);
    elseif method == "SIFT"
        featurePoints = detectSIFTFeatures(image);
    elseif method == "SURF"
        featurePoints = detectSURFFeatures(image);
    end
end