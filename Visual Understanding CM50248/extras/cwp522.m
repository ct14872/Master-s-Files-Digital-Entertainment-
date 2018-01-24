function cwp522()
    close all;
    
    n_images = 6
    n_pts_homo = 4
    for i = 1:n_images
      imageFileName = sprintf('image%d.tif', i);
      imageFileNames{i} = imageFileName;
    end
    [imagePoints,boardSize,imagesUsed] = detectCheckerboardPoints(imageFileNames); 
    boardSize
    size(imagePoints)
    imageFileNames = imageFileNames(imagesUsed);
    for i = 1:numel(imageFileNames)
      I = imread(imageFileNames{i});
      subplot(3, 3, i);
      imshow(I);
      hold on;
      plot(imagePoints(1,1,i),imagePoints(1,2,i),'ro');
      plot(imagePoints(13,1,i),imagePoints(13,2,i),'ro');
    end
    
    random_indices = [104,67,77,6]%randsample((boardSize(1)-2)*(boardSize(2)-2), 4)%[121,156,39,24]%[139,68,130,141]%[120,152,106,92]%[37,89,107,63]
    col = ones(4,1);
%     points = zeros(n_pts_homo,2,n_images);
%     for i = 1:n_images
%         points(:,:,i) = imagePoints(random_indices,:,i);
%     end  
%     size1 = size(checkpoints1)
%     size2 = size(imagePoints(:,:,1))
    
    points1 = imagePoints(random_indices,:,1);
%     points1 = [points1 col]
    points2 = imagePoints(random_indices,:,2);
%     points2 = [points2 col]
    points3 = imagePoints(random_indices,:,3);
%     points3 = [points3 col]
    
%     A = calibrate(points,n_images)%
    normed_p = translate_to_checker(boardSize);
    normed_p = normed_p(random_indices,:);
    [K,E12,E13,E23] = get_intrinsics(imageFileNames,points1,points2,points3, normed_p)    
    
    [k1,k2] = find_lens_distortion(K, E12,E13,E23,points1,points2,points3,normed_p)
    cameraParams = cameraParameters('IntrinsicMatrix',K, 'RadialDistortion', [0 0]);
    
    original1 = imread(imageFileNames{1});
    corrected1 = undistortImage(original1,cameraParams);
    figure; imshowpair(imresize(original1, 0.5),imresize(corrected1, 0.5),'montage');
    
    original2 = imread(imageFileNames{2});
    corrected2 = undistortImage(original2,cameraParams);
    figure; imshowpair(imresize(original2, 0.5),imresize(corrected2, 0.5),'montage');
    
    original3 = imread(imageFileNames{3});
    corrected3 = undistortImage(original3,cameraParams);
    figure; imshowpair(imresize(original3, 0.5),imresize(corrected3, 0.5),'montage');
    
    original4 = imread(imageFileNames{4});
    corrected4 = undistortImage(original4,cameraParams);
    figure; imshowpair(imresize(original4, 0.5),imresize(corrected4, 0.5),'montage');

end

function [vs, H] = calc_entriesV(p1,p2)
%     ransac_parameters.it_num = 1000;     %how many iterations the ransac algorithm to run
%     ransac_parameters.num_for_homo = 4;          %how many points to be used to estimate homography on each run
%     ransac_parameters.distance_thr_inlier = 1;   % the threshold deciding the fate of the point. is it an inlier or outlier?
%     H = ransac_homography(ransac_parameters,p1,p2)
    H = homography_calc(p1,p2)
%     H = H';
%     h1 = H(:,1);
%     h2 = H(:,2);
%     h3 = H(:,3);
%     
    %orthonormal constraints
    %v1 --> dot product == 0
%     v12 = [h1(1)*h1(2), h1(1)*h2(2)+ h2(1)*h1(2), h3(1)*h1(2)+h1(1)*h3(2), ... 
%                     h2(1)*h2(2),h3(1)*h2(2)+h2(1)*h3(2),h3(1)*h3(2)];
%     
%     %v2 --> their length ==1
%  
%     v11 = [h1(1)*h1(1), h1(1)*h2(1)+ h2(1)*h1(1), h3(1)*h1(1)+h1(1)*h3(1), ... 
%                     h2(1)*h2(1),h3(1)*h2(1)+h2(1)*h3(1),h3(1)*h3(1)];
%     v22 = [h1(2)*h1(2), h1(2)*h2(2)+ h2(2)*h1(2), h3(2)*h1(2)+h1(2)*h3(2), ... 
%                     h2(2)*h2(2),h3(2)*h2(2)+h2(2)*h3(2),h3(2)*h3(2)];

    v12 = con_v(H,1,2)
    
    v11 = con_v(H,1,1);
    v22 = con_v(H,2,2);

    v2 = v11-v22;
    
    vs = [v12;v2];
    
end

function v = con_v(H,i,j)
    
    v = [H(1,i)*H(1,j),H(1,i)*H(2,j)+H(2,i)*H(1,j),H(2,i)*H(2,j), H(3,i)*H(1,j)+H(1,i)*H(3,j),H(3,i)*H(2,j)+H(2,i)*H(3,j),H(3,i)*H(3,j)];    
end

% function A = calibrate(p,n)


function E = extrinsic_params(A,H)
    h1 = H(:,1);
    h2 = H(:,2);
    h3 = H(:,3);
    
    temp1 = sqrt(inv(A)*h1);
    lamda1 = 1/dot(temp1,temp1);
    
    temp2 = sqrt(inv(A)*h2);
    lamda2 = 1/dot(temp2,temp2);
    
    lamda3 = (lamda1+lamda2)/2;
    
    r1 = lamda1*inv(A)*h1;
    
    r2 = lamda2*inv(A)*h2;
    
    r3 = cross(r1,r2);
    
    t = lamda3*inv(A)*h3;
    
    E = [r1 r2 r3 t];
    
end

function H = homography_calc(points1,points2)
        [h,~] = size(points1);
        [h2,~] = size(points2);
        if h<4 || h2<4 || ~(isequal(h2,h))
            error('Function homography_est(A,B) requires that A and B must have at least 4 points, each, and be of the same size to estimate the homography matrix');
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

% translate to checker board points (i.e. normalise)
function checkPoints = translate_to_checker(b_size)
    checkPoints = [];
    for i = 0:b_size(2)-2
        for j = 0:b_size(1)-2
            checkPoints = [checkPoints; [j i]];
        end
    end
end

function [D_entries, d_entries] = calc_entriesD(K,E,normed, p)
    u0 = K(1,3);
    v0 = K(2,3);
    
    zer = zeros(size(normed,1),1);
    one = ones(size(normed,1),1);
    normed = [normed zer one];
    for i= 1:size(p,1)
        uv = (K*E)*normed(i,:)';
        uv_hat = p(i,:);
        
        a1 = (uv(1,1)-u0)*(power(normed(i,1),2)+power(normed(i,2),2));
        a2 = (uv(1,1)-u0)*power((power(normed(i,1),2)+power(normed(i,2),2)),2);
        
        b1 = (uv(2,1)-v0)*(power(normed(i,1),2)+power(normed(i,2),2));
        b2 = (uv(2,1)-v0)*power((power(normed(i,1),2)+power(normed(i,2),2)),2);
        
        D_entries = [a1 a2; b1 b2];
        
        c1 = uv_hat(1,1)- uv(1,1);
        c2 = uv_hat(1,2)- uv(2,1);
        
        d_entries = [c1;c2];
    end   
end

function [k1,k2] = find_lens_distortion(K, E1,E2,E3,p1,p2,p3,normed_p)
    
    [D_e1, d_e1] = calc_entriesD(K,E1,normed_p,p1);
    [D_e2, d_e2] = calc_entriesD(K,E2,normed_p,p2);
    [D_e3, d_e3] = calc_entriesD(K,E3,normed_p,p3);
    
    D = [D_e1; D_e2; D_e3]
    d = [d_e1; d_e2; d_e3]
    
    k = inv(D'*D)*D'*d
    
    k1 = k(1);
    k2 = k(2);

end
