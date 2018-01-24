function cwp3(p,hsize,sigma)
    close all;

    if p == 1
        
        %%%%%%%%%    Part 3.1     %%%%%%%%%
        % From Lecture slides:
        % Canny edges Algorithm in steps:
         img = im2double(imread('pola.jpg'));
        %imshow(img)
        x = rgb2gray(img);
                
        % 1. Blur image to reduce noise (hsizexhsize gaussian kernel with sd = sigma)
        gaus = fspecial('gaussian',hsize,sigma);
        f = conv2(x,gaus,'same');
                
        % 2. Calculate gradients for horizontal and vertical direction using filters below
        dx = [-1, 0, 1; -1, 0, 1; -1, 0, 1];
        dy = [1, 1, 1; 0, 0, 0; -1, -1, -1];
        
        filt_in_x = conv2(f,dx,'same');
        filt_in_y = conv2(f,dy,'same');
                    
        %Calculate directions/orientations
        dir = atan2(filt_in_y,filt_in_x);
        dir = dir*180/pi;
        
        % Calculate the magnitude of the image 
        mag = sqrt(((filt_in_x.^2) + (filt_in_y.^2)));
        
        % Show results up to now
        subplot(2,3,1); imshow(img); title('Orig. Image ')
        subplot(2,3,2); imshow(x); title('Grayscale ')
        subplot(2,3,3); imshow(f); title('Gaussian filtered ')
        subplot(2,3,4); imshow(filt_in_x); title('Filtered in x-direction')
        subplot(2,3,5); imshow(filt_in_y); title('Filtered in y-direction')
        subplot(2,3,6); imshow(mag); title('Magnitude')
        
        [h,w] = size(f);
        
        % Setting all directions to positive by giving them a full spin
        for i = 1:h
            for j = 1:w
                if dir(i,j)< 0
                    dir(i,j) = dir(i,j)+360;
                end
            end
        end
        
        % 2.1 Setting directions in bins of 45 degree multiples (0,45,90,135)
        n_dir = zeros(size(f));
        for i = 1:h
            for j = 1:w
                if ((dir(i,j) >= 0) && (dir(i,j) < 22.5) || (dir(i,j) >= 157.5) && (dir(i,j) < 202.5) || (dir(i,j) >= 337.5) && (dir(i,j) <= 360))
                    n_dir(i,j) = 0;
                elseif ((dir(i,j) >= 22.5) && (dir(i,j) < 67.5) || (dir(i,j) >= 202.5) && (dir(i,j) < 247.5))
                    n_dir(i,j) = 45;
                elseif ((dir(i,j) >= 67.5) && (dir(i,j) < 112.5) || (dir(i,j) >= 247.5) && (dir(i,j) < 292.5))
                    n_dir(i,j) = 90;    
                elseif ((dir(i,j) >= 112.5) && (dir(i,j) < 157.5) || (dir(i,j) >= 292.5) && (dir(i,j) < 337.5))
                    n_dir(i,j) = 135;
                end
            end
        end

      %  figure; imagesc(n_dir); colorbar;
        
        % 3. Non- Maximum Supression to achieve edge thining
        thin = zeros(size(f));
        for i=2:h-1 %since we are also checking the one next to it
            for j=2:w-1 %since we are checking the one above it
                if (n_dir(i,j)==0)
                    if (mag(i,j) == max([mag(i,j),mag(i,j+1),mag(i,j-1)]))
                        thin(i,j) = 1;
                    else
                        thin(i,j) = 0;
                    end
                elseif (n_dir(i,j)==45)
                    if (mag(i,j) == max([mag(i,j),mag(i-1,j+1),mag(i+1,j-1)]))
                        thin(i,j) = 1;
                    else
                        thin(i,j) = 0;
                    end
                elseif (n_dir(i,j)==90)
                    if (mag(i,j) == max([mag(i,j),mag(i+1,j),mag(i-1,j)]))
                        thin(i,j) = 1;
                    else
                        thin(i,j) = 0;
                    end
                elseif (n_dir(i,j)==135)
                    if (mag(i,j) == max([mag(i,j),mag(i+1,j+1),mag(i-1,j-1)]))
                        thin(i,j) = 1;
                    else
                        thin(i,j) = 0;
                    end
                end
            end
        end
        thin = thin.*mag;
        %figure,imshow(thin);
        
        final = zeros(size(f));
        final2 = zeros(size(f));
        high = 0.15;
        low = 0.4 * high;
        change = 1;
        % 4. Hysteresis thresholding - keep searching for edge pixels until
        % 2 continuous results are the same
        while(change)
            for i = 1:h
                for j = 1:w
                    if(thin(i,j)<low)
                        final(i,j) = 0;
                    elseif (thin(i,j)>high) 
                        final(i,j) = 1;
                    elseif (thin(i,j)>low && ((thin(i-1,j+1)>high) || (thin(i,j+1)>high) || (thin(i+1,j+1)>high) || (thin(i-1,j-1)>high) || (thin(i,j-1)>high) || (thin(i+1,j-1)>high) || (thin(i-1,j)>high) || (thin(i+1,j)>high)) )
                        thin(i,j) = 1;
                        final(i,j) = 1;
                    end
                end
            end
            
            if(final == final2)
                change = 0;
            else
                final2 = final;
            end
        end
        
        
        figure;imshow(final); title(['\sigma = ',num2str(sigma),' and kernel size = ',num2str(hsize),'x',num2str(hsize)]);
        
        %%%%%%%%% End of Part 3.1 %%%%%%%%%
        
    elseif p == 2
        
        %%%%%%%%%    Part 3.2     %%%%%%%%%
        img2 = im2double(imread('sunflowers.jpg'));
        x2 = rgb2gray(img2);
        
        gaus = fspecial('gaussian',hsize,sigma);
        
        % Horizontal and verticcal gradients
        [dx, dy] = meshgrid(-1:1,-1:1);
        
        % filtering along each direction gradient
        Ix = conv2(x2, dx, 'same');
        Iy = conv2(x2, dy, 'same');
        
        % preparing variables
        Ix_sqr = Ix.^2;
        Iy_sqr = Iy.^2;
        Ixy_mul = Ix.*Iy;
        
        % gaussian filtering 
        Ix2 = conv2(Ix_sqr, gaus,'same');
        Iy2 = conv2(Iy_sqr, gaus, 'same');
        Ixy = conv2(Ixy_mul, gaus, 'same');
        
        [h,w] = size(x2);
        
        % Finding the cornerness score
        k = 0.08;
        Mc = zeros(size(x2));
        max_val = 0;
        for i = 1:h
            for j = 1:w
                M = [Ix2(i,j) Ixy(i,j); Ixy(i,j) Iy2(i,j)];
                Mc(i,j) = det(M)-k*(trace(M))^2;
                max_val = max(max_val,Mc(i,j));
            end
        end
        
        % Non-Maximum-Suppression
        final = zeros(size(img2));
        for i = 2:h-1
            for j = 2:w-1
                if Mc(i,j) > max_val/10
                    if Mc(i,j) > Mc(i-1,j+1) && Mc(i,j) > Mc(i-1,j) && Mc(i,j) > Mc(i-1,j-1) && Mc(i,j) > Mc(i,j+1) && Mc(i,j) > Mc(i,j-1) && Mc(i,j) > Mc(i+1,j+1) && Mc(i,j) > Mc(i+1,j) && Mc(i,j) > Mc(i,j-1) 
                        final(i,j) = 1;
                    end
                end
            end
        end
        
        % Find all corner coordinates with values equal to 1 in final
        % (corners)
        [y,x] = find(final == 1);
        imshow(img2); hold on;
        plot(x,y,'go');title(['\sigma = ',num2str(sigma),' and kernel size = ',num2str(hsize),'x',num2str(hsize)]);
    %%%%%%%%% End of Part 3.2 %%%%%%%%%   
    
    elseif p ==3
        
    %%%%%%%%%    Part 3.3     %%%%%%%%%
    % Unfinished
        img2 = im2double(imread('coins.png'));
        imshow(img2);
        %x2 = rgb2gray(img2);
        for j = 1:3
            dog_batch = dog(img2,6);
            [~,~,w] = size(dog_batch);
            figure;
            for i = 1:w
                subplot(1,w,i);imshow(10*dog_batch(:,:,i));
            end
            ex_min = blob_detection(dog_batch);
            [~,~,d] = size(ex_min);
            figure;
            for i = 1:d
                subplot(1,d,i);imshow(ex_min(:,:,i));
            end
            img2 = imresize(img2,1/(2^j));
        end
        
    end
    %%%%%%%%% End of Part 3.3 %%%%%%%%%
    
end

% Find the difference of gaussians
function dog_batch = dog(img,octave_size)
    
    octave = dog_octave(img,octave_size);
    
    dog_batch = zeros(size(octave,1),size(octave,2),octave_size-1);
    
    for i = 2:octave_size
        dog_batch(:,:,i-1) = octave(:,:,i) - octave(:,:,i-1);
    end

end

% Blur each image with a different sigma and return them
function octave = dog_octave(img, num)
              
    octave = zeros(size(img,1),size(img,2),num);
    
    sigma = 1.4;
    k = 1/num;
    figure;
    for i = 1:num
        h = fspecial('gaussian',11,i*k*sigma);
        octave(:,:,i) = conv2(img,h,'same'); 
        subplot(1,num,i); imshow(octave(:,:,i));
    end
end

% This function is used to find extrema and minima around neighborhoods
function extrema_minima = blob_detection(dog)
    
    [h,w,d] = size(dog);
    
    extrema_minima = zeros(h,w,(floor(d/3)+1));
    
    for i = 2:h-1
        for j = 2:w-1
            for k = 2:d-1
                if (dog(i,j,k) == max([dog(i,j,k),dog(i-1,j,k),dog(i-1,j-1,k),dog(i,j-1,k),dog(i+1,j,k),dog(i+1,j+1,k),dog(i,j+1,k),dog(i-1,j+1,k),dog(i+1,j-1,k),...
                        dog(i,j,k-1),dog(i-1,j,k-1),dog(i-1,j-1,k-1),dog(i,j-1,k-1),dog(i+1,j,k-1),dog(i+1,j+1,k-1),dog(i,j+1,k-1),dog(i-1,j+1,k-1),dog(i+1,j-1,k-1),...
                        dog(i,j,k+1),dog(i-1,j,k+1),dog(i-1,j-1,k+1),dog(i,j-1,k+1),dog(i+1,j,k+1),dog(i+1,j+1,k+1),dog(i,j+1,k+1),dog(i-1,j+1,k+1),dog(i+1,j-1,k+1)])) 
                    extrema_minima(i,j,k-1) = 1;
                elseif( dog(i,j,k) == min([dog(i,j,k),dog(i-1,j,k),dog(i-1,j-1,k),dog(i,j-1,k),dog(i+1,j,k),dog(i+1,j+1,k),dog(i,j+1,k),dog(i-1,j+1,k),dog(i+1,j-1,k),...
                        dog(i,j,k-1),dog(i-1,j,k-1),dog(i-1,j-1,k-1),dog(i,j-1,k-1),dog(i+1,j,k-1),dog(i+1,j+1,k-1),dog(i,j+1,k-1),dog(i-1,j+1,k-1),dog(i+1,j-1,k-1),...
                        dog(i,j,k+1),dog(i-1,j,k+1),dog(i-1,j-1,k+1),dog(i,j-1,k+1),dog(i+1,j,k+1),dog(i+1,j+1,k+1),dog(i,j+1,k+1),dog(i-1,j+1,k+1),dog(i+1,j-1,k+1)]))
                    extrema_minima(i,j,k-1) = 1;
                end
            end
        end
    end

end
