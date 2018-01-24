function cwp5()

    %%%%%%%%%    Part 5 - Intrinsic Parameters and lens distortion coefficients     %%%%%%%%%
    close all;
    
    %checkerboard images taken from: http://www.vision.caltech.edu/bouguetj/calib_doc/htmls/example.html
    
    % Load images of checkerboards
    n_images = 5
    n_pts_homo = 4
    index = [1,2,3,4,7];
    for i = 1:n_images
      imageFileName = sprintf('image%d.tif', index(i));
      imageFileNames{i} = imageFileName;
    end
    
    % Find checkerboard corners
    [imagePoints,boardSize,imagesUsed] = detectCheckerboardPoints(imageFileNames); 

    imageFileNames = imageFileNames(imagesUsed);
    for i = 1:numel(imageFileNames)
      I = imread(imageFileNames{i});
      subplot(1, n_images, i);
      imshow(I);
      hold on;
      plot(imagePoints(:,1,i),imagePoints(:,2,i),'ro');
    end
    
    % normed_p are the model coordinates (only the checkboard) set as (0,0),(1,0),(2,0).....
    normed_p = translate_to_checker(boardSize);
    [A,extrinsics] = get_intrinsics_extrinsics(imagePoints, normed_p, boardSize)
    
    % find initial estimate of distortion coefficients
    [k1,k2] = find_lens_distortion(A, extrinsics,imagePoints,normed_p);
    
    % visually checking for the undistorted image
    k1 = k1/10;
    k2 = k2/100000;
    for i=1:n_images
        cameraParams = cameraParameters('IntrinsicMatrix',A, 'RadialDistortion', [k1 k2]);
        original1 = imread(imageFileNames{i});
        corrected1 = undistortImage(original1,cameraParams);
        figure; imshowpair(imresize(original1, 1),imresize(corrected1, 1),'montage');
    end
    %%%%%%%%%    Part 5 - Intrinsic Parameters and lens distortion coefficients     %%%%%%%%%
    
end

% Convert model points to a new coordinate system (0,0), (0,1), (0,2), ....
% (boardSize(1)-2, boardSize(2)-2)
function checkPoints = translate_to_checker(b_size)
    checkPoints = [];
    % Corners start from the second outer layer of the checkers and because
    % of 0 we subtract 2.
    for i = 0:b_size(2)-2
        for j = 0:b_size(1)-2
            checkPoints = [checkPoints; [j i]];
        end
    end
end

% For easier interpretation to follow the paper
function v = con_v(H,i,j)
    
    v = [H(1,i)*H(1,j),H(1,i)*H(2,j)+H(2,i)*H(1,j),H(2,i)*H(2,j), ...
            H(3,i)*H(1,j)+H(1,i)*H(3,j),H(3,i)*H(2,j)+H(2,i)*H(3,j),H(3,i)*H(3,j)];    
end

% construction of the small v terms as seen in Zhang's paper
function [vs, H] = calc_entriesV(p1,p2)

    H = homography_calc(p1,p2);

    v12 = con_v(H,1,2);
    
    v11 = con_v(H,1,1);
    v22 = con_v(H,2,2);

    v2 = v11-v22;
    
    vs = [v12;v2];
    
end

% Get intrinsic and extrinsic parameters
function [A, extrinsics] = get_intrinsics_extrinsics(impoints, normed,boardSize)

    n_images = size(impoints,3);
    V_mat = [];
    Homographies = zeros(3,3,n_images);
    random_indices = [75,102,83,90];%randsample((boardSize(1)-1)*(boardSize(2)-1), 4)%[154,156,122,4]%[153,26,133,113]%[125,139,32,15]%[75,102,83,90]
    for i = 1:n_images
        % calculates homographies and entries to v matrix
        [v,H] = calc_entriesV(impoints(random_indices,:,i),normed(random_indices,:));
        Homographies(:,:,i) = H;
        % stack vs
        V_mat = [V_mat;v];
    end
    
    % unit vector associated with the smallest singular value of V_mat
    [~,~,V] = svd(V_mat);
    % b = [B11,B12,B22,B13,B23,B33]
    b = V(:,end);
    B11 = b(1);
    B12 = b(2);
    B22 = b(3);
    B13 = b(4);
    B23 = b(5);
    B33 = b(6);

    % Formulas from paper:
    
    % y coordinate of principal point
    v0 = ((B12*B13)-(B11*B23))/((B11*B22)-power(B12,2));
    
    %lambda
    lambda = B33-((power(B12,2)+v0*((B12*B13)-(B11*B23)))/B11);

    % a
    alpha = sqrt(lambda/B11);
    
    % b
    temp = (lambda*B11)/(B11*B22-power(B12,2));
    beta = sqrt(temp);
    
    %gamma -> skewness
    gamma = (-B12*power(alpha,2)*beta)/lambda;
    
    % x coordinate of principal points
    u0 = ((gamma*v0)/beta)-((B13*power(alpha,2))/lambda);
    
    % Intrinsic parameter matrix A
    A = [alpha,gamma,u0;0,beta,v0;0,0,1];
    
    % calculate the extrinsics for each viewpoint
    extrinsics = zeros(3,4,n_images);
    for j = 1:size(Homographies,3)
        E = extrinsic_params(A,Homographies(:,:,j));
        extrinsics(:,:,j) = E;
    end
 
end

function E = extrinsic_params(A,H)
% Following the formulas in the paper
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
    
    % E = [R|t]
    E = [r1 r2 r3 t];
    
end

% Calculate homography from 4 point correspondences
function H = homography_calc(points1,points2)

        [h,~] = size(points1);
        [h2,~] = size(points2);
        [points1, Tp1] = normalise_points(points1);
        [points2, Tp2] = normalise_points(points2);
        size(points1);
        size(points2);
        
        %Check if at least four points and if they are of the same size
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
        
        
        % Unnormalisation
        H = inv(Tp2)*H*Tp1;
        H = (1/H(end,end))*H;
end

% Normalises points to be zero centred and at distance of max sqrt(2) from
% origin. Also returns the normalisation matrices to be used to unnormalise
% a matrix.
function [normal_p, T_p] = normalise_points(p)

    n = size(p,1);
    mu_p = mean(p);
    zero_centred = [p(:,1)-mu_p(1) p(:,2)-mu_p(2)];
    sd_p = std(zero_centred);
    
    T_sd_p = [sqrt(2)/sd_p(1),0,0;0,sqrt(2)/sd_p(2),0; 0,0,1];
    T_mu_p = [1,0,-mu_p(1); 0,1,-mu_p(2);0,0,1];
    
    T_p = T_sd_p * T_mu_p;
    
    normal_p = T_p * [p'; ones(1,n)];
    normal_p = normal_p';

end

% Calculates the entries for D and d find the radial distortion
% coefficients for each image.
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

% calculates lens distortion coefficients as seen in Zhangs paper
% under the method of alternation
function [k1,k2] = find_lens_distortion(K, Es,points,normed_p)
    
    D = [];
    d = [];
    for i=1:size(points,3)
        [Dt,dt] = calc_entriesD(K,Es(:,:,i),normed_p,points(:,:,i));
        D = [D;Dt];
        d = [d;dt];
    end
    
    k = inv(D'*D)*D'*d
    
    k1 = k(1);
    k2 = k(2);

end

