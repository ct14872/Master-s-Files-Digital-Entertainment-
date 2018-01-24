function cwp2(p)
    close all;
    img = im2double(imread('pola.jpg'));
    x = rgb2gray(img);
    img2 = rgb2gray(imread('lake.jpg'));
    img3 = rgb2gray(imread('mustang.jpg'));    
    
    if p==1
    %%%%%%%%%    Part 2.1     %%%%%%%%%
        mask = [1 1 1; 1 1 1; 1 1 1]; %kernel
        h = (1/(sum(sum(abs(mask))))).*mask;
        h2 = fspecial('gaussian',11,2.4);
       
        y = basic_convolution(x,h);
        f = conv2(x,h,'same');
        
        y2 = basic_convolution(x,h2);
        f2 = conv2(x,h2,'same');
        
        temp = abs(f-y);
        ssd = sum(temp(:).^2)
        
        temp2 = abs(f2-y2);
        ssd2 = sum(temp2(:).^2)
        
        subplot(2,2,1); imshow(img); title('Original Image');
       	subplot(2,2,2); imshow(y); title('Basic Convolution');
        subplot(2,2,4); imshow(f); title('MATLAB''S conv2');
        subplot(2,2,3); imshow(x); title('Grayscale Image');
%        	subplot(2,3,3); imshow(y2); title('Basic Convolution (gaussian)');
%         subplot(2,3,6); imshow(f2); title('MATLAB''S conv2 (gaussian)');        
    %%%%%%%%% End of Part 2.1 %%%%%%%%%
    
    elseif p==2
    %%%%%%%%%    Part 2.2     %%%%%%%%%
        mask = [1 1 1; 1 1 1; 1 1 1]; %kernel
        h = (1/(sum(sum(abs(mask))))).*mask;
        h2 = fspecial('gaussian',11,2.4);
        
        y = extended_convolution(x,h);
        f = imfilter(x,h,'replicate','conv');
        
        y2 = extended_convolution(x,h2);
        f2 = imfilter(x,h2,'replicate','conv');
        
        temp = abs(y-f);
        ssd = sum(temp(:).^2) 
        
        temp2 = abs(f2-y2);
        ssd2 = sum(temp2(:).^2)
        
        subplot(2,2,1); imshow(img); title('Original Image');
       	subplot(2,2,2); imshow(y); title('Extended Convolution');
        subplot(2,2,4); imshow(f); title('MATLAB''S imfilter');
        subplot(2,2,3); imshow(x); title('Grayscale Image');
%        	subplot(2,3,5); imshow(y2); title('Extended Convolution (gaussian)');
%         subplot(2,3,6); imshow(f2); title('MATLAB''S imfilter (gaussian)');
    %%%%%%%%% End of Part 2.2 %%%%%%%%%
    
    elseif p==3
    %%%%%%%%%    Part 2.3     %%%%%%%%%

        subplot(2,2,1); imshow(x); title('Using extended convolution:');
%         subplot(2,4,5); imshow(x); title('Using built-in imfilter:');
        
        j = 1;
        for i = 1:3
            h2 = compute_gaussian(11,i^3)% fspecial('gaussian', 11,i^3);%compute_gaussian(3, i^3);
            y = extended_convolution(x, h2);
            f = imfilter(x,h2,'replicate','conv');
            
            subplot(2,2,j+1); imshow(y); title(['\sigma = ',num2str(i^3)])
%             subplot(2,2,j+5); imshow(f); title(['\sigma = ',num2str(i^3)])
            
            temp = abs(f-y);
            ssd = sum(temp(:).^2)
            j = j+1;
        end
        
        vert = [1 0 -1; 1 0 -1; 1 0 -1]; %vertical
        hor = [1 1 1; 0 0 0; -1 -1 -1]; %horizontal
        diag = [1 1 0; 1 0 -1; 0 -1 -1]; % diagonal
        unsharp_mask = [0 -1 0; -1 5 -1; 0 -1 0]; %krnel for unsharp masking
        
        vertical = extended_convolution(x,vert);
        horizontal = extended_convolution(x,hor);
        diagonal = extended_convolution(x,diag);
        un_masked = extended_convolution(x,unsharp_mask);
        
        figure; subplot(2,2,1); imshow(vertical);title('Vertical');
        subplot(2,2,2); imshow(horizontal);title('Horizontal');
        subplot(2,2,3); imshow(diagonal);title('Diagonal');
        subplot(2,2,4); imshow(un_masked);title('Unsharp Masking');
    %%%%%%%%% End of Part 2.3 %%%%%%%%%
   
    elseif p==4
    %%%%%%%%%    Part 2.4     %%%%%%%%%
        turns = 5;
        points1 = zeros(turns,2);
        points2 = zeros(turns,2);
        for j = 1:turns
            h2 = compute_gaussian(j*2+1,2.4);
            total = 0;
            for i = 1:3
                tic;
                f = extended_convolution(x, h2);
                conv_time = toc;
                total = total + conv_time;
            end
            Avrg_excon_time = total/3;
            points1(j,:) = [((j*2)+1) Avrg_excon_time];

            total2 = 0;
            for i = 1:3
                tic;
                y = fft_convolution(x, h2);
                fft_conv_time = toc;
                total2 = total2 + fft_conv_time;
            end
            Avrg_fft_time = total2/3;
            points2(j,:) = [((j*2)+1) Avrg_fft_time];

            j=j+1;
        end
               
        subplot(1,3,1); imshow(img); title('Original Image');
        subplot(1,3,2); imshow(y); title('Convoled using FFT');
        subplot(1,3,3); imshow(f); title('Extended Convolution');
        
%         figure, subplot(1,2,1),plot((points1(:,1)).^2,points1(:,2),'b'); grid on; hold on; 
%         plot((points2(:,1)).^2,points2(:,2),'r'); legend('Ext conv','FFT conv'); hold off;
%         xlabel('Kernel Area(Gaussian)');
%         ylabel('Avrg time taken (s)');
%         title('Extended Convolution Vs. FFT Convolution');
        
        figure; plot(points1(:,1),points1(:,2),'b'); grid on; hold on; 
        plot(points2(:,1),points2(:,2),'r'); legend('Ext conv','FFT conv'); hold off;
        xlabel('Side of square kernel (Gaussian)');
        ylabel('Avrg time taken (s)');
        title('Extended Convolution Vs. FFT Convolution'); 
    end
    %%%%%%%%% End of Part 2.4 %%%%%%%%%
end  

% Padded with zeros throughout and then goes through the convolution
% filtering
function y = basic_convolution( x, h)

    x2 = padarray(x,[floor(size(h,1)/2) floor(size(h,2)/2)],0,'both'); %padded with zeros on all sides
    y = zeros(size(x2));
    
    for i = (size(h,1)+1)/2:size(x2,1)-floor(size(h,1)/2)
        for j = (size(h,2)+1)/2:size(x2,2)-floor(size(h,2)/2)
            for m = 1:size(h,1)
                for n = 1:size(h,2)
                    y(i,j) = y(i,j) + (x2((i-m+(size(h,1)+1)/2),(j-n+(size(h,2)+1)/2)).*h(m,n));
                end
            end
        end
    end
    
    % get image back to original size(as conv2(img,h,'same') would do)
    y = y((size(h,1)+1)/2:size(y,1)-floor(size(h,1)/2), (size(h,2)+1)/2:size(y,2)-floor(size(h,2)/2));
    
end

% Main difference is that here the img is centered by replicating the outer
% most layers instead of padding it with zeros as before
function y = extended_convolution( x, h)
    %tic;
    pad_x = padarray(x,[floor(size(h,1)/2) floor(size(h,2)/2)],'both','replicate');
    y = basic_convolution( pad_x, h);
    y = y((size(h,1)+1)/2:size(y,1)-floor(size(h,1)/2), (size(h,2)+1)/2:size(y,2)-floor(size(h,2)/2));
    %t = toc
end

% Used to compute gaussian kernels based on the side and standard
% deviation given as input.
function gaus = compute_gaussian(side,sigma)
    gaus = ones([side side]);
    for m = -floor(size(gaus,1)/2):floor(size(gaus,1)/2)
        for n = -floor(size(gaus,1)/2):floor(size(gaus,1)/2)
            gaus(m+ceil(size(gaus,1)/2),n+ceil(size(gaus,2)/2)) = (1/(2*pi*sigma^2))*exp(-(m^2 + n^2)/(2*(sigma^2)));
        end
    end
    %normalize
    gaus = (1/(sum(sum(abs(gaus))))).*gaus;
end

function fft_conv = fft_convolution(x,h)
    % Do necessary padding to center image, padding depends on kernel size
    % and image size as well.
    [pm, pn] = size(x);
    [pmm, pnn] = size(h);
    
    fm = pm + pmm -1;
    fn = pn + pnn -1;
    
    pad_x = padarray(x,[floor(size(h,1)/2) floor(size(h,2)/2)],'both','replicate');
    
    % Convolution in the fourier domain is translated as simple
    % multiplication
    fft_conv = ifft2(fft2(pad_x,fm,fn).*fft2(h,fm,fn));
    
    % Remove padded pixel layers
    padRy = ceil((pmm-1)./2);
    padRx = ceil((pnn-1)./2);
    fft_conv = fft_conv(padRy+1:pm+padRy,padRx+1:pn+padRx); 
end
