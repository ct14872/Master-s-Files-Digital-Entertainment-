function cwp4(p)
    close all;
      % Demo for 4.1
    img1 = im2double(imread('vase1.jpg'));
    img2 = im2double(imread('vase2.jpg'));

      % Demo for 4.2 and 4.3
%     img1 = im2double(imread('parthenon.jpg'));
%     img2 = im2double(imread('parthenon2.jpg'));
      
      % Demos for 4.4
%       img1 = im2double(imread('mustang1.jpg'));
%       img2 = im2double(imread('mustang.jpg'));
%     img1 = im2double(imread('eiffel.jpg'));
%     img2 = im2double(imread('eiffel2.jpg'));

    % The variable distorted is used throughout my implementation, but it
    % actually means a different perspective image

    % Please uncomment this if you want to see the results when the images
    % are resized and rotated
%     img1 = imresize(img1,1.5);
%     img1 = imrotate(img1,27);

    imshowpair(img2,img1,'montage');
    title('The images chosen for demo.');
    
    original = rgb2gray(img1);
    distorted = rgb2gray(img2);

    % Detect image features (SURF - Speeded Up Robust Features) and
    % their points with respect to the image.
    original_pts = detectSURFFeatures(original);
    distorted_pts = detectSURFFeatures(distorted);

    % Extract the feature descriptors with their respective valid
    % points on the image.
    [original_feat_descriptors, original_valid] = extractFeatures(original, original_pts);
    [distorted_feat_descriptors, distorted_valid] = extractFeatures(distorted, distorted_pts);
    if p ==1
        
        %%%%%%%%%    Part 4.1     %%%%%%%%%
        [h,~] = size(original_feat_descriptors);
        [h2,~] = size(distorted_feat_descriptors);

        % Finding matching features using: 1) Euclidean Distance (2 minimum distances) and 2) Applying the 
        % ratio test to avoid unique matches (1st run)
        index_pairs = zeros(h,2);     
        for i=1:h
            [first_min,second_min,index] = min_distance(original_feat_descriptors(i,:), distorted_feat_descriptors);
            r = (first_min/second_min);
            if r<0.8
                index_pairs(i,1) = i;
                index_pairs(i,2) = index;
            end
        end
        
        % Finding matching features using: 1) Euclidean Distance (2 minimum distances) and 2) Applying the 
        % ratio test to avoid unique matches (2nd run)       
        index_pairs2 = zeros(h2,2);     
        for i=1:h2
            [first_min,second_min,index] = min_distance(distorted_feat_descriptors(i,:), original_feat_descriptors);
            r = (first_min/second_min);
            if r<0.8
                index_pairs2(i,1) = i;
                index_pairs2(i,2) = index;
            end
        end
        
        % Discarding non-mutually exclusive matches by comparing the two
        % index_pairs lists
        final_index_pairs = zeros(max(h,h2),2);
        for i = 1:h
            for j = 1:h2
                if index_pairs(i,1) == index_pairs2(j,2) && index_pairs(i,2) == index_pairs2(j,1)
                    final_index_pairs(i,:) = index_pairs(i,:);
                end
            end
        end
                
        final_index_pairs = final_index_pairs(any(final_index_pairs,2),:);
        
        matched1 = original_valid(final_index_pairs(:,1));
        matched2 = distorted_valid(final_index_pairs(:,2));
        
        figure; showMatchedFeatures(img1,img2,matched1,matched2,'montage');
        %%%%%%%%% End of Part 4.1 %%%%%%%%%  
        
    elseif p==2
        
        %%%%%%%%%    Part 4.2     %%%%%%%%%
        index_pairs = matchFeatures(original_feat_descriptors,distorted_feat_descriptors);
        
        matched1 = original_valid(index_pairs(4:7,1));
        matched2 = distorted_valid(index_pairs(4:7,2));
        points1 = matched1.Location;
        points2 = matched2.Location;
        H = homography_calc(points1,points2);
                
        % The following code is for part 4.3 the first task.
        % Input with mouse the coordinates of the point desired to be
        % matched.
        figure; showMatchedFeatures(img1,img2,matched1,matched2,'montage');title('Points chosen to compute homography matrix');
        figure;imshow(img1); hold on; title('Select a point on the original image to match on the corresponding distorted image');
        
        [mx, my] = ginput(1);
        plot(mx,my,'go','LineWidth',5,'MarkerSize',10);
        
        % Use homography matrix to estimate the position of the point
        % inputted using the mouse, on the distorted image.
        input_point = [mx my 1];
        p_second_img = input_point*H;
        p_second_img = (1/p_second_img(3))*p_second_img;
        
        figure; imshow(img2);hold on; plot(p_second_img(1),p_second_img(2),'rx','LineWidth',5,'MarkerSize',10);title('Corresponding point on distorted image');
        %%%%%%%%% End of Part 4.2 %%%%%%%%%
        
    elseif p==3
        
        %%%%%%%%%    Part 4.3     %%%%%%%%%
        index_pairs = matchFeatures(original_feat_descriptors,distorted_feat_descriptors);
        
        % Manually picked matches for homography estimation
        matched1 = original_valid(index_pairs(4:7,1));
        matched2 = distorted_valid(index_pairs(4:7,2));
        points1 = matched1.Location;
        points2 = matched2.Location;
        H = homography_calc(points1,points2)
        
        mapped = imgwarping(img1,img2,H);
        figure; subplot(2,1,1);imshowpair(img2, mapped,'montage'); title('Backward Mapping without Bilinear Interpolation');
        
        mapped2 = imgwarping2(img1,img2,H);
        subplot(2,1,2);imshowpair(img2, mapped2,'montage'); title('Backward Mapping with Bilinear Interpolation');
        %%%%%%%%% End of Part 4.3 %%%%%%%%%
        
    elseif p==4
        
        %%%%%%%%%    Part 4.4     %%%%%%%%%
        index_pairs = matchFeatures(original_feat_descriptors,distorted_feat_descriptors);
        
        matched1 = original_valid(index_pairs(:,1));
        matched2 = distorted_valid(index_pairs(:,2));
        points1 = matched1.Location;
        points2 = matched2.Location;
        
        ransac_parameters.it_num = 1000;     %how many iterations the ransac algorithm to run
        ransac_parameters.num_for_homo = 4;          %how many points to be used to estimate homography on each run
        ransac_parameters.distance_thr_inlier = 1;   % the threshold deciding the fate of the point. is it an inlier or outlier?
        
        H = ransac_homography(ransac_parameters, points1, points2);
        
        mapped = imgwarping2(img1,img2,H);
        figure; imshowpair(img2,mapped,'montage'); title('Backward Mapping with Bilinear Interpolation - montage');
        figure; imshowpair(img2,mapped,'blend'); title('Backward Mapping with Bilinear Interpolation - blend');
        %%%%%%%%% End of Part 4.4 %%%%%%%%%
        
    end
end

% This function iterates over descriptors, finding the 2 minimum distances
% and their respectives indices. (Used for Brute Force Feature matching)
function [first_min,second_min,index] = min_distance(this_descriptor, descriptors)
    [h,~] = size(descriptors);
    distances = zeros(h,2);
    for i = 1:h
        distances(i,1) = norm(this_descriptor-descriptors(i,:));
        distances(i,2) = i;
    end
    distances = sortrows(distances,1);
    first_min = distances(1,1);
    second_min = distances(2,1);
    index = distances(1,2);
end

% Homogeneous solution for homography matrix estimation
% Based on https://moodle.bath.ac.uk/pluginfile.php/966584/mod_resource/content/6/CM20219%20Course%20Notes%202017-8%20%28version%202017-10-06%29.pdf
% and http://www.robots.ox.ac.uk/~vgg/presentations/bmvc97/criminispaper/node3.html
function H = homography_calc(points1,points2)
    [h,~] = size(points1);
    [h2,~] = size(points2);
    if h<4 || h2<4 || ~(isequal(h2,h))
        error('Function homography_calc(A,B) requires that A and B must have at least 4 points, each, and be of the same size to estimate the homography matrix');
    end
    
    A = zeros(2*h,9);
    
    % Easy construction of the A matrix
    for i = 1:h
         A((2*i-1),:) = [points1(i,1), points1(i,2), 1, 0, 0, 0, -points1(i,1)*points2(i,1), -points2(i,1)*points1(i,2), -points2(i,1)];
         A((2*i),:)   = [0, 0, 0, points1(i,1), points1(i,2), 1, -points1(i,1)*points2(i,2), -points2(i,2)*points1(i,2), -points2(i,2)];
    end

    [~,~,V] = svd(A);
    h=V(:,end);

    H = reshape(h,3,3);
    H = (1/H(end,end))*H;
end

% Image warping from source to target given the homography transformation
% matrix. This function uses backward mapping (mapping from source to target) to create the output.
function transformed = imgwarping(src,trgt, H)
    transformed = zeros(size(trgt));
    for i = 1:size(trgt, 1)
        for j = 1:size(trgt, 2)

            point = [j i 1];
            p_prime =  point/H;% Equivalent to point*inv(H), proposed by matlab
            p_prime = (1/p_prime(3))*p_prime;
            y = round(p_prime(1));
            x = round(p_prime(2));

            if (y > 0 && x > 0 && x <= size(src, 1) && y <= size(src, 2) )
                transformed(i, j, :) = src(x, y, :);
            end
        end
    end
end

function transformed = imgwarping2(src,trgt, H)
    transformed = zeros(size(trgt));
    for i = 1:size(trgt, 1)
        for j = 1:size(trgt, 2)

            point = [j i 1];
            p_prime = point/H;% Equivalent to point*inv(H), proposed by matlab
            p_prime = (1/p_prime(3))*p_prime;
            y = p_prime(1);
            x = p_prime(2);
            
            if (isinteger(x) && isinteger(y)) % if both x and y coordinates are integers, just set the corresponding pixel
                transformed(i, j, :) = src(x, y, :);
            else % else perform bilinear interpolation 
                x_left = floor(x);
                y_down = floor(y);
                x_right = ceil(x);
                y_up = ceil(y);
                
                dis_left = x-x_left;
                dis_down = y-y_down;
                dis_right = 1-dis_left;
                dis_up = 1-dis_down;
                
                if (y_down > 0 && x_left > 0 && x_right <= size(src, 1) && y_up <= size(src, 2) )
                    % As seen from the lecture slides
                    f1 = src(x_left,y_up,:);
                    f2 = src(x_right,y_up,:);
                    f3 = src(x_left,y_down,:);
                    f4 = src(x_right,y_down,:);
                    f12 = dis_right*f1 + dis_left*f2;
                    f34 = dis_right*f3 + dis_left*f4;
                    f1234 = dis_down*f12 + dis_up*f34;
                    transformed(i, j, :) = f1234;
                end
            end
        end
    end
end

function H = ransac_homography(params, feat_points1, feat_points2)
   % based on https://moodle.bath.ac.uk/pluginfile.php/850436/mod_resource/content/6/CM50248%20Notes%20%282017-10-01%29.pdf
    feat_num = size(feat_points1, 1);
    best_num_inliers = 0;
    best_inlier_indices = [];
    for iter = 1: params.it_num
       
        % Step 1a: Randomly choose at least 4 points to calculate
        % homography.
        random_indices = randsample(feat_num, params.num_for_homo);
        points1 = feat_points1(random_indices,:);
        points2 = feat_points2(random_indices,:);
        
        % Step 1b: Compute model using points chosen randomly
        curr_H = homography_calc(points1,points2);
        
        % Step 1c: Fit the model on the data to further check for
        % percentage of inliers
        inlier_indices = zeros(feat_num);
        index = 1;
        for i = 1:feat_num
                if(ismember(i,random_indices))
                    continue
                else

                    point = [feat_points1(i,:) 1];
                    p_prime =  point*curr_H;% Equivalent to point*inv(curr_H), proposed by matlab
                    p_prime = (1/p_prime(3))*p_prime;

                    dis_x = p_prime(1) - feat_points2(i,1);
                    dis_y = p_prime(2) - feat_points2(i,2);

                    curr_ssd = (dis_x^2 + dis_y^2); 

                    if (curr_ssd <= params.distance_thr_inlier)
                        inlier_indices(index) = i;
                        index = index +1;
                    end
                end
        end
        inlier_indices = inlier_indices(any(inlier_indices,2));
        numb_of_inliers = length(inlier_indices);
        
        % Step 2: Get the highest quality matcher (i.e the most inliers)
        if (numb_of_inliers > best_num_inliers)
            best_num_inliers = numb_of_inliers
            best_inlier_indices = inlier_indices;
        end   
    end
    
    % Step 3: Optionally refit model to all inliers to improve model
    new_points1 = feat_points1(best_inlier_indices,:);
    new_points2 = feat_points2(best_inlier_indices,:);
    H = homography_calc(new_points1,new_points2);
end

